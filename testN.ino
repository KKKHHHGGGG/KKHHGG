#include <SPI.h>
#include <MFRC522.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>

const int RST_PIN = 5;
const int SS_PIN = 53;
const int MAGNETIC_SENSOR_PIN = 7;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 10;

std_msgs::String rfid_msg;
ros::NodeHandle nh;
ros::Publisher Rfid("ros_to_firebase", &rfid_msg);

MFRC522 mfrc(SS_PIN, RST_PIN);
Servo servo2;
Servo servo3;

int magnetic_sensorValue;
bool open_state = false;

void appCallback(const std_msgs::String& msg)
{
  if(strcmp(msg.data, "Open") == 0){  
    open_state = true;
  }
}


ros::Subscriber<std_msgs::String> servo("firebase_to_ros", appCallback);

void setup() {
  initHardware();
  initROS();
  openDoor();
}

void loop() {
  nh.spinOnce();
  magnetic_sensorValue = digitalRead(MAGNETIC_SENSOR_PIN);
  
  processDoorState();
  processRFID();
  delay(10);
}

void initHardware() {
  SPI.begin();
  mfrc.PCD_Init();
  pinMode(MAGNETIC_SENSOR_PIN, INPUT_PULLUP);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
}

void initROS() {
  nh.initNode();
  nh.advertise(Rfid);
  nh.subscribe(servo);
}

void processDoorState() {
  if (open_state) {
    openDoor();
    if (magnetic_sensorValue == LOW) {
      open_state = false;
    }
  }
}

void processRFID() {
  if (magnetic_sensorValue == HIGH && open_state == false) {
    if (!mfrc.PICC_IsNewCardPresent() || !mfrc.PICC_ReadCardSerial()) {
      delay(50);
      return;
    }
    else if (mfrc.uid.uidByte[0] == 67 && mfrc.uid.uidByte[1] == 213 &&
             mfrc.uid.uidByte[2] == 143 && mfrc.uid.uidByte[3] == 2)
    {
      if (rfid_msg.data != "Hotel_Mode") {
        rfid_msg.data = "Hotel_Mode";
        Rfid.publish(&rfid_msg);
        delay(500);
      }
    }
    else if (mfrc.uid.uidByte[0] == 117 && mfrc.uid.uidByte[1] == 44 &&
             mfrc.uid.uidByte[2] == 176 && mfrc.uid.uidByte[3] == 137)
    {
      if (rfid_msg.data != "Other_Mode") {
        rfid_msg.data = "Other_Mode";
        Rfid.publish(&rfid_msg);
        delay(500);
      }
    }
    else {
      if (rfid_msg.data != "None") {
        rfid_msg.data = "None";
        Rfid.publish(&rfid_msg);
        delay(500);
      }
    }
  } else {
    if (rfid_msg.data != "None") {
      rfid_msg.data = "None";
      Rfid.publish(&rfid_msg);
      delay(500);
    }
  }

  if (rfid_msg.data == "Hotel_Mode" || rfid_msg.data == "Other_Mode") {
    delay(5000);
    closeDoor();
  }

  delay(10);
}


void closeDoor() {
  servo2.write(30);
  servo3.write(30);
}

void openDoor() {
  servo2.write(130);
  servo3.write(130);
}
