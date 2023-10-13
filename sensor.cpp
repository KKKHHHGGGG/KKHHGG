#include <string>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <wiringPi.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#define Trig 15 // BCM 14 (WiringPi 15)
#define Echo 26 // BCM 12 (WiringPi 26)
#define Trig1 16 // BCM 14 (WiringPi 15)
#define Echo1 27 // BCM 12 (WiringPi 26)

using namespace std;

class Sensor{
public:
    Sensor(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pubUltraEnd = nh.advertise<std_msgs::String>("/ultra", 1);

        sub_turn_ultra = nh.subscribe("/ultra", 1, &Sensor::subUltra, this);
    }

    // 초음파 작동 //
    void ultra_move(string direction) {

        int start_time, end_time;
        float distance1, distance2;

        while (ros::ok())
        {
            digitalWrite(Trig, LOW);
            digitalWrite(Trig1, LOW);
            delay(100);
            digitalWrite(Trig, HIGH);
            digitalWrite(Trig1, HIGH);
            delayMicroseconds(10);
            digitalWrite(Trig, LOW);
            digitalWrite(Trig1, LOW);

            while (digitalRead(Echo) == 0 && digitalRead(Echo1) == 0);
            start_time = micros();

            while (digitalRead(Echo) == 1 && digitalRead(Echo1) == 1);
            end_time = micros();

            distance1 = (end_time - start_time) / 29. / 2.;
            distance2 = (end_time - start_time) / 29. / 2.;  // Assuming distance2 is obtained from the second ultrasonic sensor

            cout << "[INFO] Distance 1: " << distance1 << " cm, Distance 2: " << distance2 << " cm" << endl;

            if (direction == "forward") {
                if (distance1 > 12.5 && distance2 > 12.5) {
                    cmd_vel.linear.x = 0.06;
                    pubCmdvel.publish(cmd_vel);
                }
                else {
                    cmd_vel.linear.x = 0.00;
                    pubCmdvel.publish(cmd_vel);
                    ROS_INFO("STOP");
                    break;
                }
            }
        }
    }


    void exit_ev(string direction) {
        if (direction == "ex_ev") {
            cmd_vel.linear.x = 0.10;
        }

        ROS_INFO("Exit_Elevator");

        for (int i = 0; i < 1010; i++) {
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.linear.x = 0.00;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Exit_end");
    }
    // 90도 회전 //
    void turn90deg(string direction) {
        if (direction == "left") {
            cmd_vel.angular.z = 0.8;
        }

        if (direction == "right") {
            cmd_vel.angular.z = -0.8;
        }

        ROS_INFO("Turn 90deg start");

        for (int i = 0; i < 206; i++) {
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 90deg end");
    }
    // 180도 회전 //
    void turn180deg(string direction) {
        if (direction == "left") {
            cmd_vel.angular.z = 0.8;
        }
        
        if (direction == "right") {
            cmd_vel.angular.z = -0.8;
        }

        ROS_INFO("Turn 180deg start");

        for (int i = 0; i < 403; i++) {
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 180deg end");
    }

    void subUltra(const std_msgs::String ultra) {
        // 90도 회전
        if (strcmp(ultra.data.c_str(), "right90") == 0 ) {
            ROS_INFO("90turn");
            turn90deg("right");
        }

        // 180도 회전
        if (strcmp(ultra.data.c_str(), "right180") == 0 ) {
            ROS_INFO("180turn");
            turn180deg("right");
        }

        // 초음파 전진
        if (strcmp(ultra.data.c_str(), "exit") == 0 ) {
            ROS_INFO("Exit_EV");
            exit_ev("ex_ev");
            delay(500);
            ROS_INFO("eixt_complete");
            end.data= "complete";
            pubUltraEnd.publish(end);
        }

        // 초음파 전진 후 180도 회전
        if (strcmp(ultra.data.c_str(), "EVin") == 0) {
            ROS_INFO("in to EV");
            ultra_move("forward");
            delay(500);
            ROS_INFO("180turn");
            turn180deg("left");
            delay(500);
            end.data= "end";
            pubUltraEnd.publish(end);
        }
    }


private:
    ros::NodeHandle nh;
    
    ros::Publisher pubCmdvel;
    ros::Publisher pubUltraEnd;

    ros::Subscriber sub_turn_ultra;

    geometry_msgs::Twist cmd_vel;
    std_msgs::String end;

    inline void delay(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }
    inline void delayMicrosecond(int us) { this_thread::sleep_for(chrono::microseconds(us)); }
};

int main(int argc, char**argv){
    ros::init(argc, argv, "Sensor");


    if (wiringPiSetup() == -1) {
        ROS_ERROR("Failed to initialize WiringPi.");
        return 1;
    }

    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    
    ROS_INFO("SET UP ULTRA SENSOR");

    Sensor sensor;

    ros::spin();

    return 0;
}
