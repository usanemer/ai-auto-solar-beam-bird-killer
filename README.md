# AI 자동 태양광 조류퇴치기
-실시간으로 AI가 조류를 인식하고, 인식된 조류 위치로 태양빛을 집광하여 조류를 퇴치하는 시스템.   
**Languages**: Python, C  
**AI Frameworks**: TensorFlow Lite, PyTorch, OpenCV  
**Embedded Systems**: Raspberry Pi , Arduino  
**Protocols**: SSH, VNC  

[시연영상](https://drive.google.com/file/d/1IfoJf7-g2vfZvFv16jHVQO_M5lm80YaR/view?usp=sharing)  


## 파일 설명
best-fp16.tflite : 조류 탐지 모델  
SolarBeam_ArduinoUNO.ino : 아두이노 코드  
SolarBeam_RaspberryPi.py : 라즈베리파이 코드  


```mermaid
graph TD
    A([Start Loop]) --> B[태양 위치 계산]
    B --> C{태양 고도 > 0?}
    
    C -- No (Night) --> D[집광 중단]
    C -- Yes (Day) --> E[Capture Image<br>&<br>Run YOLOv5 TFLite 조류탐지 모델]
    
    E --> F{조류 탐지?}
    F -- No --> D
    F -- Yes --> G[가장 가까운 조류를 타겟으로 설정]
    
    G --> H[좌표계 변경<br>Camera -> Horizon -> West-Tilted]
    H --> I[집광기 & 반사기 각도 계산]
    I --> J[계산된 데이터를 17-byte Stream 형태로 변환]
    
    D --> A
    J --> K[17바이트 데이터를 아두이노에 전송]
    K --> L[Arduino: 6개의 모터 제어]
    L --> A
