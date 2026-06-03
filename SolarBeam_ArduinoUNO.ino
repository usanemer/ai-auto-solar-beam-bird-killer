#include <Servo.h>
#define BPS 9600
Servo servo_mirror_az;              //반사기 방위 모터
Servo servo_mirror_alt;             //반사기 고도 모터
Servo servo_concentrator_az;        //집광기 방위 모터
Servo servo_concentrator_alt1;      //집광기 고도 모터1
Servo servo_concentrator_alt2;      //집광기 고도 모터2
Servo servo_concentrate_controller; //집광통제기 모터


bool stop_lightCollection;    //집광 통제 여부
float mirror_az;              //반사기 방위 모터 회전각
float mirror_alt;             //반사기 고도 모터 회전각
float concentrator_az;        //집광기 방위 모터 회전각
float concentrator_alt;       //집광기 고도 모터 회전각
float concentrator_alt1;      //집광기 1번 고도 모터 회전각
float concentrator_alt2;      //집광기 2번 고도 모터 회전각
float concentrate_controller; //집광 통제기 회전각

void setup() {
  Serial.begin(BPS);
  //Servo.h라이브러리는 소프트웨어적으로 PWM 신호를 생성해주기 때문에 일반 디지털 핀에서도 모터를 구동할 수 있다.
  servo_mirror_az.attach(3);                 
  servo_mirror_alt.attach(4);
  servo_concentrator_az.attach(5);
  servo_concentrator_alt1.attach(6);
  servo_concentrator_alt2.attach(7);
  servo_concentrate_controller.attach(8);
}

void loop() {
  //17바이트 데이터를 라즈베리파이로 부터 수신
  if (Serial.available() >= 17) {
    byte buffer[17];
    int readBytes = Serial.readBytes(buffer, 17);
    if(readBytes == 17){//연속된 17바이트 데이터를 해독한다.
      memcpy(&stop_lightCollection, buffer,     1);
      memcpy(&mirror_alt,           buffer + 1, 4);
      memcpy(&mirror_az,            buffer + 5, 4);
      memcpy(&concentrator_alt,     buffer + 9, 4);
      memcpy(&concentrator_az,      buffer + 13,4);
      
      if(stop_lightCollection)//집광 통제 여부에 따라 집광통제 모터 각도 조절
        concentrate_controller = 90;
      else
        concentrate_controller = 0;
        
        //집광기 고도조절 모터 각도 계산
        if(concentrator_alt > 180){
          concentrator_alt1 = 180;
          concentrator_alt2 = concentrator_alt - 180;
        }else{
          concentrator_alt1 = concentrator_alt;
          concentrator_alt2 = 0;
        }

        // 전압 강하 방지를 위해 금속 모터들을 회전시킬때 0.5초의 간격을 두고 회전시킨다.
        servo_mirror_az.write(mirror_az);
        servo_mirror_alt.write(mirror_alt);
        servo_concentrator_az.write(concentrator_az);
        delay(500);//0.5초 지연
        servo_concentrator_alt1.write(concentrator_alt1);
        delay(500);
        servo_concentrator_alt2.write(concentrator_alt2);
        servo_concentrate_controller.write(concentrate_controller);
      
    }
  }
}
