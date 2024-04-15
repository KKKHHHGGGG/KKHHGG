#include <AccelStepper.h>

#define STEP_PIN_1 30 // 스텝 모터 1의 스텝 핀
#define DIR_PIN_1 32  // 스텝 모터 1의 방향 핀
#define ENABLE_PIN 34

AccelStepper stepper1(1, STEP_PIN_1, DIR_PIN_1); // 스텝 모터 1에 대한 AccelStepper 인스턴스 생성

void setup() {
  Serial.begin(9600); // 시리얼 통신 시작
  stepper1.setMaxSpeed(4000); // 스텝 모터 1의 최대 속도 설정
  stepper1.setAcceleration(2000); // 스텝 모터 1의 가속도 설정
  pinMode(ENABLE_PIN, OUTPUT); // ENABLE_PIN을 출력으로 설정
  digitalWrite(ENABLE_PIN, LOW); // 스텝 모터 활성화
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString(); // 시리얼에서 문자열 읽기
    command.trim(); // 문자열의 양쪽 끝 공백 제거
    if (command == "go") { // "go" 문자열을 받았는지 확인
      // 스텝 모터 1을 전진시킵니다.
      stepper1.setSpeed(1100); // 스텝 모터 1의 속도 설정
      stepper1.move(500); // 스텝 모터 1을 500 스텝만큼 전진시킵니다.
      stepper1.runToPosition(); // 목표 위치에 도달할 때까지 스텝 모터 1을 실행합니다.
      
      delay(2500); // 2.5초 동안 대기합니다.
      
      // 스텝 모터 1을 후진시킵니다.
      stepper1.setSpeed(-1100); // 스텝 모터 1의 속도 설정
      stepper1.move(-500); // 스텝 모터 1을 500 스텝만큼 후진시킵니다.
      stepper1.runToPosition(); // 목표 위치에 도달할 때까지 스텝 모터 1을 실행합니다.
      
      delay(2500); // 2.5초 동안 대기합니다.
    }
  }
}
