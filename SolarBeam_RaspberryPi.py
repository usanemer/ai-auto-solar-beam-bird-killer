import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2
import time

import struct
import serial
import math

from dataclasses import dataclass

# ==========================================
# 1. 태양 위치 및 좌표 변환 관련 함수 정의
# ==========================================

def solar_time_min(N,longitude, timezone, t_clock_min):
"""
    지방 표준시(Clock Time)를 태양시(Solar Time)로 변환 (단위: 분)
    - N: 연중 경과일 (1월 1일 = 1)
    - longitude: 경도
    - timezone: 표준 시간대 (예: 한국 = +9)
    - t_clock_min: 현재 시각을 분으로 환산한 값
    """
    B = 2*math.pi*(N-81)/364
    EoT = 9.87*math.sin(2*B) - 7.53*math.cos(B) - 1.5*math.sin(B)
    
    long_correction = 4 * (longitude - timezone * 15)

    t_solar_min = t_clock_min + long_correction + EoT
    return t_solar_min


#태양 위치 반환. 방위각은 북쪽을 0도로 시계방향(위에서 볼경우)(북,동,남,서)으로 진행
def sun_position_clock_min(N, lat, longitude, timezone, t_clock_min):
    latitude = math.radians(lat)

    delta = math.radians(23.45) * math.sin( 2*math.pi*(N+284)/365 )


    t_solar_min = solar_time_min(N,longitude, timezone, t_clock_min)

    H = math.radians(0.25) * (t_solar_min - 720)

    h = math.asin(
        math.sin(latitude)*math.sin(delta) +
        math.cos(latitude)*math.cos(delta)*math.cos(H)
    )

    A = math.atan2(
        math.sin(H),
        math.cos(H)*math.sin(latitude) - math.tan(delta)*math.cos(latitude)
    )

    r = 1.0
    az = (math.degrees(A) + 180) % 360
    alt = math.degrees(h)

    return r, az, alt



# ==========================================
# 2. 영상 좌표계 기반 조류 위치 분석 함수
# ==========================================


#카메라에 찍힌 조류 방위각 반환(시계방향)
def angle_with_y_axis(x, y, imageWidth, imageHeight):
    """
    이미지의 중심(카메라 광학축)을 기준으로 객체의 방위각(시계방향 편차) 계산
    """
    rel_x = x - imageWidth/2
    rel_y = y - imageHeight/2

    if rel_x == 0 and rel_y == 0:
        return 0
    elif rel_x == 0:
        if rel_y > 0:
            return 0
        else:
            return math.pi
    elif rel_y == 0:
        if rel_x > 0:
            return math.pi / 2
        else:
            return -math.pi / 2

    az = math.degrees(math.atan2(rel_x, -rel_y))
    return az

#새의 고도 계산
def pixel_to_total_angle_precise(x, y, fov_h, fov_v, image_width, image_height):
    cx = image_width / 2
    cy = image_height / 2

    az_x = (x - cx) / cx * (fov_h / 2)
    az_y = (y - cy) / cy * (fov_v / 2)

    # 라디안 변환
    tx = math.tan(math.radians(az_x))
    ty = math.tan(math.radians(az_y))

    # 정확한 공간각 (라디안 → 도)
    az_total = math.degrees(math.atan(math.sqrt(tx**2 + ty**2)))
    return az_total



# ==========================================
# 3. 3차원 기하학적 좌표계 회전 함수 (로드리게스 회전 공식 공식 활용)
# ==========================================

#좌표계 회전 함수
def horiz_to_vector(A_deg, h_deg):
    """방위각(A)과 고도(h)를 3차원 직교좌표계 단위 벡터(x, y, z)로 변환"""
    A = math.radians(A_deg)
    h = math.radians(h_deg)
    x = math.cos(h) * math.sin(A)
    y = math.cos(h) * math.cos(A)
    z = math.sin(h)
    return (x, y, z)

# 벡터 v를 축 u를 중심으로 angle_deg만큼 회전. 지평좌표계의 좌표를를 모터를 기준으로하는 모터좌표계로 변환
def rotate_about_axis(v, u, angle_deg):
    ax = math.radians(angle_deg)
    ux, uy, uz = u
    norm = math.sqrt(ux*ux + uy*uy + uz*uz)
    ux, uy, uz = ux/norm, uy/norm, uz/norm

    vx, vy, vz = v
    cosA = math.cos(ax)
    sinA = math.sin(ax)
    # cross product (u × v)
    cx = uy*vz - uz*vy
    cy = uz*vx - ux*vz
    cz = ux*vy - uy*vx
    dot = ux*vx + uy*vy + uz*vz

    rx = vx*cosA + cx*sinA + ux*dot*(1 - cosA)
    ry = vy*cosA + cy*sinA + uy*dot*(1 - cosA)
    rz = vz*cosA + cz*sinA + uz*dot*(1 - cosA)
    return (rx, ry, rz)

def vector_to_horiz(v):
    """3차원 직교좌표 벡터(x, y, z)를 방위각과 고도로 변환"""
    x, y, z = v
    A = math.degrees(math.atan2(x, y)) % 360
    z_clamped = max(-1.0, min(1.0, z))
    h = math.degrees(math.asin(z_clamped))
    return A, h


#실제 사용할 함수
def convert_zenith_tilt_west(az_deg, alt_deg, tilt_deg=145.0):
    """
    천정(Zenith) 기준 지평좌표를 서쪽으로 기울어진 모터 구조물 좌표계로 변환
    - tilt_deg=145.0: 모터 마운트가 기울어진 각도
    """
    # 회전축 = 북쪽(y축)
    axis_north = (0.0, 1.0, 0.0)
    v = horiz_to_vector(az_deg, alt_deg)
    # 좌표계를 +tilt 회전 → 점은 -tilt 회전
    v_rot = rotate_about_axis(v, axis_north, -tilt_deg)
    return vector_to_horiz(v_rot)



# ==========================================
# 4. 하드웨어 연결 및 초기화 설정
# ==========================================


ser = None
def connectUNO(): 
    """아두이노(UNO) 직렬 통신(Serial) 포트 자동 탐색 및 연결 함수"""
    #init serial communication 
    global ser 
    if not ser or not ser.is_open: 
        try: 
            ser = serial.Serial('/dev/ttyACM0', 9600) 
            print("Connected to /dev/ttyACM0") 
        except serial.SerialException: 
            try: 
                ser = serial.Serial('/dev/ttyACM1', 9600) 
                print("Connected to /dev/ttyACM1") 
            except serial.SerialException: 
                print("Failed to connect to both /dev/ttyACM0 and /dev/ttyACM1") 
                ser = None 
                time.sleep(2)   

# 객체 검출용 변환 모델(TFLite) 로드 및 텐서 할당
model_path = "/home/won/best-fp16.tflite"
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
img_size = input_details[0]['shape'][1]# 모델 요구 입력 크기(예: 640 또는 320)

# 파이카메라 2 객체 생성 및 구동
picam = Picamera2()
picam.stop()
picam.start()
time.sleep(1)

# 이전 프레임에서 추적된 빛/객체의 정규화 좌표 초기값 (중앙)
current_light_pos = (0.5,0.5)

connectUNO()

# 사용 중인 카메라 모듈의 물리적 화각(FOV) 데이터
cameraFOV_horizon = 62.2
cameraFOV_vertical = 48.8

@dataclass
class TimeNPlace:
    """시간 및 위치 데이터 구조체"""
    latitude: float
    longitude:float
    timezone: int
    N:int
    t_clock_min:int



# 가상 테스트 환경 시나리오 정의
in_zenith = TimeNPlace(latitude=0.18, longitude=-78.5, timezone=-5, N=80, t_clock_min=12*60)  # 키토 인근 천정 상태
in_south = TimeNPlace(latitude=37.5, longitude=127, timezone=9, N=315, t_clock_min=13*60)    # 서울 오후 상태
in_west = TimeNPlace(latitude=37.5, longitude=127, timezone=9, N=266, t_clock_min=18*60)     # 서울 일몰 상태

times = [in_zenith, in_south, in_west]


#전역 상태 변수 정의

#태양의 지평좌표계상 좌표
#alt = 고도, az = 방위각
horizon_coordination_sun_alt_angle = 0
horizon_coordination_sun_az_angle = 0
#태양의 모터 좌표계상 좌표
tilt_west_coordination_sun_alt_angle = 0
tilt_west_coordination_sun_az_angle = 0
#새의 영상좌표계상 좌표
image_coordination_bird_alt = 0
image_coordination_bird_az = 0
#새의 지평좌표계상 좌표
horizon_coordination_bird_alt = 0
horizon_coordination_bird_az = 0
#새의 모터 좌표계상 좌표
tilt_west_coordination_bird_alt = 0
tilt_west_coordination_bird_az = 0

#반사기 각도
mirror_alt_angle = 0
mirror_az_angle = 0
#집광기 각도
Concentrator_alt_angle = 0
Concentrator_az_angle = 0

stop_lightCollection = True # 집광 중지 플래그

print("=======================================")
print("        BIRD KILLER ACTIVATED")
print("=======================================")


# ==========================================
# 5. 메인 제어 루프
# ==========================================
while(True):
    mode = 0  # 시나리오 모드 선택 (0: in_zenith, 1: in_south, 2: in_west)

    # 1) 현재 시간/위치 기준 태양의 지평좌표(방위각, 고도) 계산
    (r, horizon_coordination_sun_az_angle, horizon_coordination_sun_alt_angle) = sun_position_clock_min(
        times[mode].N, 
        times[mode].latitude, 
        times[mode].longitude, 
        times[mode].timezone, 
        times[mode].t_clock_min
    )
    
    print(f"solar az:{horizon_coordination_sun_az_angle}")
    print(f"solar alt:{horizon_coordination_sun_alt_angle}")
    
    # 2) 카메라 이미지 캡처
    image = picam.capture_array()
    
    # 야간 (태양 고도가 0도 이하일 경우 집광 중단)
    if horizon_coordination_sun_alt_angle <= 0:
        print("stop light collection")
        stop_lightCollection = True

    else: # 주간 (태양 고도가 0도 이상인 경우 집광)
        stop_lightCollection = False
        
        # 알파 채널 제거 및 RGB 배열 정규화 변환
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        camera_image_width = image.shape[1]
        camera_image_height = image.shape[0]
        
        # 3) YOLOv5 TFLite 추론 전처리 및 실행
        img_resized = cv2.resize(image, (img_size, img_size))
        img_input = img_resized.astype(np.float32) / 255.0
        img_input = np.expand_dims(img_input, axis=0)

        interpreter.set_tensor(input_details[0]['index'], img_input)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]['index'])[0] # 결과 텐서 확보 (배열 형태: [25200, 6])

        # 4) 조류 검출, 타깃 조류 선정
        conf_threshold = 0.3
        birds = []
        for det in output: # 검출된 모든 조류들과 현재 시스템의 초점간 거리 계산
            x, y, w, h, conf, cls_id = det
            if conf > conf_threshold:
                distance_m2 = (x - current_light_pos[0])**2 + (y - current_light_pos[1])**2
                birds.append((x, y, distance_m2))

        if not birds:  # 타깃 미검출 시 레이저/집광 중단
            print("Nooo bird detected.")
            stop_lightCollection = True 
            
        else:
            # 현재 시스템의 초점과 가장 가까운 객체를 타깃 조류로 지정 (최근접 이웃 추적 알고리즘)
            min_distance_bird = min(birds, key=lambda t: t[2])
            current_light_pos = min_distance_bird[0:2]

            x = int(min_distance_bird[0] * image.shape[1]) # 정규화 좌표를 픽셀 스케일로 복원
            y = int(min_distance_bird[1] * image.shape[0])
            
            cv2.circle(image, (x, y), 10, [255, 0, 0], 2) # 검출된 조류 위치 시각화 표시

            # 5) 검출 객체의 카메라 영상 기준 기하학적 각도 산출
            image_coordination_bird_az = angle_with_y_axis(x, y, camera_image_width, camera_image_height)
            image_coordination_bird_alt = pixel_to_total_angle_precise(
                current_light_pos[0] * camera_image_width,
                current_light_pos[1] * camera_image_height,
                cameraFOV_horizon,
                cameraFOV_vertical,
                camera_image_width,
                camera_image_height
            )

            # 카메라 장착 방향 오프셋에 맞춰 지평좌표계 값으로 매핑 보정
            horizon_coordination_bird_az = -(image_coordination_bird_az - 90)
            horizon_coordination_bird_alt = image_coordination_bird_alt - 90

            # 6) 계산된 모든 좌표를 모터 구동축 물리 좌표계(West-Tilted)로 최종 변환
            tilt_west_coordination_bird_az, tilt_west_coordination_bird_alt = convert_zenith_tilt_west(
                horizon_coordination_bird_az, horizon_coordination_bird_alt, 145)
            tilt_west_coordination_sun_az_angle, tilt_west_coordination_sun_alt_angle = convert_zenith_tilt_west(
                horizon_coordination_sun_az_angle, horizon_coordination_sun_alt_angle, 145)

            print(f"bird tilt:{tilt_west_coordination_bird_alt}, {tilt_west_coordination_bird_az}")
            print(f"solar tilt:{horizon_coordination_sun_alt_angle}, {tilt_west_coordination_sun_az_angle}")
            
            # 7) 반사 법칙(입사각=반사각)에 기반한 반사경(Mirror) 벡터의 사잇각(중간값) 연산
            mirror_alt_angle = (tilt_west_coordination_bird_alt + tilt_west_coordination_sun_alt_angle) / 2
            mirror_az_angle  = (tilt_west_coordination_bird_az  + tilt_west_coordination_sun_az_angle ) / 2

            # 8) 집광기(Concentrator) 기계 각도 제어 연산
            Concentrator_az_angle = (tilt_west_coordination_sun_az_angle + 90) % 360
            Concentrator_alt_angle = (360 - tilt_west_coordination_sun_alt_angle + 35) % 360
          
            # 구조적 하드웨어 구동 범위(0~180도 가동 영역) 조정 마스킹 작업
            mirror_az_angle = mirror_az_angle % 180
            mirror_alt_angle = (360 - mirror_alt_angle + 90) % 180  

            # 태양의 방위각이 180도를 넘어간다면 180도 가동범위를 갖는 방위 조절모터의 한계를 극복하기 위해
            # 방위각에서 180도를 빼고 360도의 가동범위를 갖는 고도조절 모터를 추가 회전시킨다.
            if Concentrator_az_angle >= 180:
                Concentrator_az_angle -= 180
                gap = 125 - Concentrator_alt_angle
                Concentrator_alt_angle += gap * 2

    # 영상 출력 및 화면 갱신. 테스트(데모)용 화면
    cv2.imshow("YOLOv5 TFLite Result", image)
    if cv2.waitKey(1000) == ord('q'): # 'q' 키 누를 시 루프 탈출
        break
    
    # 9) 전력문제로 연결이 끊기는것을 방지하기 위해 매번 직렬 통신시 재연결을 실시한다.
    connectUNO()
    
    # 계산한 모터 회전각도를 2진 데이터 형식으로 우노 보드에 전송
    # 1바이트 Boolean 구조와 4바이트 실수형(Float) 4개를 바이너리로 패킹 (총 17바이트 데이터 스트림)
    data = struct.pack('<?ffff', 
                       stop_lightCollection, 
                       mirror_alt_angle, 
                       mirror_az_angle, 
                       Concentrator_alt_angle, 
                       Concentrator_az_angle)
    ser.write(data) # 시리얼 버퍼로 데이터 출력
    
    print(
        f"stop : {stop_lightCollection}"
        f"\nmirror alt : {mirror_alt_angle}"
        f"\nmirror az : {mirror_az_angle}"
        f"\nconcentrator alt : {Concentrator_alt_angle}" 
        f"\nconcentrator az : {Concentrator_az_angle}"
    )

# ==========================================
# 6. 프로그램 종료 및 자원 해제 절차
# ==========================================

# 프로그램 종료시 모든 모터를 0도로 회전시킨다.
connectUNO()
data = struct.pack('<?ffff', True, 0, 0, 0, 0)
ser.write(data)
time.sleep(10)

print("========================================")
print("                 STOP")
print("========================================")

# 시스템 리소스 반환
cv2.destroyAllWindows()
picam.stop()
