#include <L298Drv.h>
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <pm2008_i2c.h>


// 상수 정의
const int STATE_UNLOCKED = 0;
const int STATE_LOCKED = 1;
const int STATE_LOCKED_DONE = 2;
const int Fan_ON = 1;
const int Fan_OFF = 0;
uint32_t lastMeasurementTime = 0;  // 변수 추가: 마지막 측정 시간

const int MOTOR_SPEED_HIGH = 160;
const int MOTOR_SPEED_LOW = 100;
const int MOTOR_DELAY = 100;  // 모터 동작 시간 (ms)
const int LED_DELAY = 50;     // LED 동작 시간 (ms)

const int GOOD_THRESHOLD = 16;
const int NORMAL_THRESHOLD = 36;
const int BAD_THRESHOLD = 76;

L298Drv motor1(8, 26);
L298Drv motor2(9, 27);
L298Drv motor3(10, 28);
L298Drv motor4(11, 29);
L298Drv motorfan(12, 30); // 미세먼지 팬

PM2008_I2C pm2008_i2c;

const int magnetic_sensorPin1 = 4;
const int magnetic_sensorPin2 = 5;
const int magnetic_sensorPin3 = 6;
const int magnetic_sensorPin4 = 7;

#define LED_PIN 13        // Neopixel 데이터 핀
#define LED_COUNT 60      // LED 스트립에 있는 전체 LED 개수

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

std_msgs::String state_msg;
std_msgs::String dust_msg;

unsigned long motorStartTime = 0;
unsigned long ledStartTime = 0;

int Fstate = -1;
int Sstate = -1;
int prev_state = -1;
int fan_state = -1;

ros::NodeHandle nh;

ros::Publisher State("state", &state_msg); 
ros::Publisher Dust("ros_to_firebase", &dust_msg);

void callback(const std_msgs::Int32& msg) {
  switch (msg.data) {
    case 0:
      Fstate = STATE_UNLOCKED;
      break;
    case 1:
      Fstate = STATE_LOCKED;
      break;
    case 2:
      Sstate = STATE_UNLOCKED;
      break;
    case 3:
      Sstate = STATE_LOCKED;
      break;
    case 4:
      fan_state = Fan_ON;
      break;
    case 5:
      fan_state = Fan_OFF;
      break;
    case 6:
      colorWipe(strip.Color(0, 50, 0, 0), 50); 
      strip.show();
      delay(1000);
      colorWipe(strip.Color(0, 0, 0, 0), 50); 
      strip.show();
      break;
    case 7:
      strip.fill(strip.Color(0, 0, 50, 0));
      strip.show();
      break;
    case 8:
      strip.clear();
      strip.show();
    case 9:
      strip.fill(strip.Color(0, 0, 0, 20));
      strip.show();
      break;
    case 10:
      strip.clear();
      strip.show();
      delay(500);
      strip.fill(strip.Color(0, 0, 0, 20));
      strip.show();
      delay(500);
      strip.clear();
      strip.show();
      delay(500);
      strip.fill(strip.Color(0, 0, 0, 20));
      strip.show();
      break;
  }
}

ros::Subscriber<std_msgs::Int32> controlSub("led_control", &callback);

void setup() {
  pinMode(magnetic_sensorPin1, INPUT_PULLUP);
  pinMode(magnetic_sensorPin2, INPUT_PULLUP);
  pinMode(magnetic_sensorPin3, INPUT_PULLUP);
  pinMode(magnetic_sensorPin4, INPUT_PULLUP);
  motor1.drive(0);
  motor2.drive(0); // 모터 정지
  motor3.drive(0);
  motor4.drive(0);
  motorfan.drive(0);

  nh.initNode();
  nh.subscribe(controlSub);
  nh.advertise(State); // 퍼블리셔 초기화
  nh.advertise(Dust);

  strip.begin();
  strip.show();  // 초기에 LED 스트립을 모두 꺼진 상태로 설정
#ifdef PM2008N

  delay(10000);
#endif
  pm2008_i2c.begin();
  pm2008_i2c.command();
  delay(1000);
}

void dust() {
// 미세먼지 코드
  if (fan_state == Fan_ON) {
    uint32_t currentTime = millis();  // 미세먼지 센서를 10초마다 측정
    if (currentTime - lastMeasurementTime >= 10000) {
      lastMeasurementTime = currentTime;  // 현재 시간으로 업데이트
      if (ret == 0) {
        int motorSpeed = 0;  // motorSpeed를 여기로 이동
        if (pm1_um >= 0 && pm1_um < GOOD_THRESHOLD) {
          dust_msg.data = "GOOD";
          motorSpeed = 0;
        } else if (pm1_um >= GOOD_THRESHOLD && pm1_um < NORMAL_THRESHOLD) {
          dust_msg.data = "NORMAL";
          motorSpeed = 0;
        } else if (pm1_um >= NORMAL_THRESHOLD && pm1_um < BAD_THRESHOLD) {
          dust_msg.data = "BAD";
          motorSpeed = 100;
        } else {
          dust_msg.data = "VERY BAD";
          motorSpeed = 200;
        }

        Dust.publish(&dust_msg);
        motorfan.drive(motorSpeed);  // motorSpeed를 설정 후 모터 구동
      }
    } 
  }else if (fan_state == Fan_OFF) {
    int motorSpeed = 0;  // motorSpeed를 여기로 이동
    motorfan.drive(motorSpeed);  // motorSpeed를 설정 후 모터 구동
  }
}

void loop() {
  
  int magnetic_sensorValue1 = digitalRead(magnetic_sensorPin1);
  int magnetic_sensorValue2 = digitalRead(magnetic_sensorPin2);
  int magnetic_sensorValue3 = digitalRead(magnetic_sensorPin3);
  int magnetic_sensorValue4 = digitalRead(magnetic_sensorPin4);

  uint8_t ret = pm2008_i2c.read();
  int pm1_um = pm2008_i2c.number_of_1_um;  
  dust();


  // 캐비넷 작업 코드

  if (Fstate != prev_state) {
    if (Fstate == STATE_UNLOCKED) {
      if (magnetic_sensorValue1 == HIGH) {
        motorStartTime = millis();
        ledStartTime = millis();
        while (magnetic_sensorValue2 == LOW) {
          if (millis() - motorStartTime >= MOTOR_DELAY) {
            motor1.drive(MOTOR_SPEED_HIGH);
            motor2.drive(-MOTOR_SPEED_HIGH); // 전진 동작 (값은 조정 가능)
            motorStartTime = millis();
          }
          if (millis() - ledStartTime >= LED_DELAY) {
            setColor(strip.Color(0, 255, 0, 0)); // 초록색 설정
            ledStartTime = millis();
          }
          magnetic_sensorValue2 = digitalRead(magnetic_sensorPin2); // magnetic_sensorPin2의 상태 읽기
        }
        motor1.drive(MOTOR_SPEED_LOW);
        motor2.drive(-MOTOR_SPEED_LOW);
        delay(500);
        state_msg.data = "Unlock1_done";
        motor1.drive(0);
        motor2.drive(0); // 전진 멈춤
        colorWipe(strip.Color(0, 0, 0, 0), 50); // LED 끄기
        Fstate = STATE_LOCKED_DONE;
      }
      State.publish(&state_msg); // state_msg 퍼블리시
      delay(10);
    }
    else if (Fstate == STATE_LOCKED) {
      motorStartTime = millis();
      ledStartTime = millis();
      while (magnetic_sensorValue1 == LOW) {
        if (millis() - motorStartTime >= MOTOR_DELAY) {
          motor1.drive(-MOTOR_SPEED_HIGH);
          motor2.drive(MOTOR_SPEED_HIGH); // 후진 동작 (값은 조정 가능)
          motorStartTime = millis();
        }
        if (millis() - ledStartTime >= LED_DELAY) {
          setColor(strip.Color(255, 165, 0, 0)); // 주황색 설정
          ledStartTime = millis();
        }
        magnetic_sensorValue1 = digitalRead(magnetic_sensorPin1); // magnetic_sensorPin1의 상태 읽기
      }
      motor1.drive(-MOTOR_SPEED_LOW);
      motor2.drive(MOTOR_SPEED_LOW);
      delay(500);
      motor1.drive(0);
      motor2.drive(0); // 후진 멈춤
      colorWipe(strip.Color(0, 0, 0, 0), 50); // LED 끄기
      state_msg.data = "Lock1_done";  
      strip.fill(strip.Color(50, 0, 0, 0));
      strip.show();
      State.publish(&state_msg); // state_msg 퍼블리시
      delay(2000);
      strip.clear();
      strip.show();

      Fstate = -1; // 상태를 -1로 변경하여 조건문이 참이 되지 않도록 함      
      delay(10);
    }

  }
    
  prev_state = Fstate;
  
  if (Sstate != prev_state) {
    if (Sstate == STATE_UNLOCKED) {
      if (magnetic_sensorValue3 == HIGH) {
        motorStartTime = millis();
        ledStartTime = millis();
        while (magnetic_sensorValue4 == LOW) {
          if (millis() - motorStartTime >= MOTOR_DELAY) {
            motor3.drive(MOTOR_SPEED_HIGH);
            motor4.drive(-MOTOR_SPEED_HIGH); // 전진 동작 (값은 조정 가능)
            motorStartTime = millis();
          }
          if (millis() - ledStartTime >= LED_DELAY) {
            setColor(strip.Color(0, 255, 0, 0)); // 초록색 설정
            ledStartTime = millis();
          }
          magnetic_sensorValue4 = digitalRead(magnetic_sensorPin4); // magnetic_sensorPin4의 상태 읽기
        }
        motor3.drive(MOTOR_SPEED_LOW);
        motor4.drive(-MOTOR_SPEED_LOW);
        delay(500);
        state_msg.data = "Unlock2_done";
        motor3.drive(0); // 전진 멈춤
        motor4.drive(0);
        colorWipe(strip.Color(0, 0, 0, 0), 50); // LED 끄기
        Sstate = STATE_LOCKED_DONE;
      }
      State.publish(&state_msg); // state_msg 퍼블리시
      delay(10);
    }
    else if (Sstate == STATE_LOCKED) {
      motorStartTime = millis();
      ledStartTime = millis();
      while (magnetic_sensorValue3 == LOW) {
        if (millis() - motorStartTime >= MOTOR_DELAY) {
          motor3.drive(-MOTOR_SPEED_HIGH);
          motor4.drive(MOTOR_SPEED_HIGH); // 후진 동작 (값은 조정 가능)
          motorStartTime = millis();
        }
        if (millis() - ledStartTime >= LED_DELAY) {
          setColor(strip.Color(255, 165, 0, 0)); // 파란색 설정
          ledStartTime = millis();
        }
        magnetic_sensorValue3 = digitalRead(magnetic_sensorPin3); // magnetic_sensorPin3의 상태 읽기
      }
      motor3.drive(-MOTOR_SPEED_LOW);
      motor4.drive(MOTOR_SPEED_LOW);
      delay(500);
      motor3.drive(0);
      motor4.drive(0); // 후진 멈춤
      colorWipe(strip.Color(0, 0, 0, 0), 50); // LED 끄기
      state_msg.data = "Lock2_done";  
      strip.fill(strip.Color(50, 0, 0, 0));
      strip.show();
      State.publish(&state_msg); // state_msg 퍼블리시
      delay(2000);
      strip.clear();
      strip.show();

      Sstate = -1; // 상태를 -1로 변경하여 조건문이 참이 되지 않도록 함 
      delay(10);          
    }
  }
  nh.spinOnce();
  delay(10);  
  prev_state = Sstate;
}

// Neopixel 색상 설정 함수
void setColor(uint32_t color) {
  for(int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// Neopixel 차례대로 색상 띄우기 함수
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}
