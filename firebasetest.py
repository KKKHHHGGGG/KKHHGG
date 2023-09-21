#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import signal
import sys
from time import sleep

# 파이어베이스
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from collections import deque

def signal_handler(sig, frame):
    print("Exiting...")
    sys.exit(0)
    
# 파이어베이스 설정
class Listener:
    

    
    def __init__(self):
        rospy.init_node('rosFirebase', anonymous=True)
        rospy.Subscriber("ros_to_firebase", String, callback=self.callback)
        self.pub = rospy.Publisher("firebase_to_ros", String, queue_size=1000)
        self.credentials_path = credentials_path
        self.database_url = database_url
        self.data_queue = deque()
        
        # Key Certification
        self.cred = credentials.Certificate(credentials_path)

        firebase_admin.initialize_app(self.cred, {
            'databaseURL': database_url  # Database url
        })
        self.hehe = None  # 변수 초기화
        self.first_order_received = True  # 변수 초기화
        
    def callback(self, message):
        ref = db.reference('')
        Order_reb = db.reference('Order')
        firstorder_ref = Order_reb.child('First_order')
        secondorder_ref = Order_reb.child('Second_order')
        if message.data in ["Hotel_Mode", "Other_Mode", "None"]:
            module_data = {"Module_Mode": message.data }
            ref.child('module').set(module_data)
        elif message.data in ["GOOD", "NORMAL", "BAD", "VERY BAD"]:
            dust_data = {"Dust_State": message.data}
            ref.child('dust').set(dust_data)
        elif message.data in ["Unlock1_done", "Unlock2_done", "Lock1_done", "Lock2_done"]:
            cabinet_data = {"Cabinet": message.data}
            ref.update(cabinet_data)
            if message.data in ["Lock1_done", "Lock2_done"]:
                next = "Next"
                first_data = firstorder_ref.get()
                second_data = secondorder_ref.get()  
                first_order_state = first_data.split('_')[1] if len(first_data.split('_')) > 1 else None
                second_order_state = second_data.split('_')[1] if len(second_data.split('_')) > 1 else None
                if first_order_state == "arrive":
                    sleep(3)
                    Order_reb.child('First_order').set(next)
                    cabinet_data = {"Cabinet": "STANBY"}
                    ref.update(cabinet_data)
                if second_order_state == "arrive":
                    sleep(3)
                    Order_reb.child('Second_order').set(next)
                    cabinet_data = {"Cabinet": "STANBY"}
                    ref.update(cabinet_data)
                    
        elif message.data in ["101_arrive", "102_arrive", "201_arrive", "202_arrive"]:
            # Extract the number from the arrive message
            number_cur = int(message.data.split('_')[0])
            first_data = firstorder_ref.get()
            second_data = secondorder_ref.get()
            self.pub.publish("drive_done")

            if first_data in ["101_go", "102_go", "201_go", "202_go"]:
                first_number_prev = int(first_data.split('_')[0])
                if number_cur == first_number_prev:
                    
                    firstorder_ref.set(message.data)
                    
            if second_data in ["101_go", "102_go", "201_go", "202_go"]:
                second_number_prev = int(second_data.split('_')[0])
                if number_cur == second_number_prev:
                    
                    secondorder_ref.set(message.data)
        else:
            data = {"Order": message.data}
            rospy.loginfo("Received data from Firebase: %s", data)
            ref.child('Order').child('hihi').set(message.data)


            
    def cabinet(self):
        cabinet = db.reference('Cabinet')
        self.cabinet_data = cabinet.get()  # 변수에 데이터 저장     
        if self.cabinet_data in ["First_Open", "First_Close", "Second_Open", "Second_Close"]:
            rospy.loginfo("Received data from cabinet: %s", self.cabinet_data)
            self.pub.publish(self.cabinet_data)
            
    def Module(self):
        module = db.reference('module')
        self.module_mode = module.child('Module_Mode').get()
        if self.module_mode == "Hotel_Mode":
            rospy.loginfo("Received data from Module: %s", self.module_mode)
            self.pub.publish(self.module_mode)    
            
    def Dust(self):
        Dust = db.reference('dust')
        self.Dust_data = Dust.child('Dust_State').get()  # 변수에 데이터 저장     
        if self.Dust_data in ["Dust_ON", "Dust_OFF"]:
            rospy.loginfo("Received data from Dust: %s", self.Dust_data)
            self.pub.publish(self.Dust_data)
    
    def drive_order(self):
        order = db.reference('Order')  
           
        self.first_order_data = order.child('First_order').get()       
        self.second_order_data = None       
                 
        if self.first_order_data in ["101_go", "102_go", "201_go", "202_go", "Next"]:
            rospy.loginfo("Received data from First_order: %s", self.first_order_data)
            if self.first_order_data in ["101_go", "102_go", "201_go", "202_go"]:
                
                self.pub.publish(self.first_order_data)

            # Read data from Second_order only when first_order_data is "Next"
            elif self.first_order_data == "Next":
                
                self.second_order_data = order.child('Second_order').get() 
                if self.second_order_data in ["101_go", "102_go", "201_go", "202_go"]:
                    rospy.loginfo("Received data from Second_order: %s", self.second_order_data)
                    self.pub.publish(self.second_order_data)
                elif self.second_order_data in ["101_arrive", "102_arrive", "201_arrive", "202_arrive"]:
                    rospy.loginfo("Received data from Second_order: %s", self.second_order_data)
                    self.pub.publish(self.second_order_data)
                    
            if self.first_order_data == "Next" and self.second_order_data == "Next":
                rospy.loginfo("home")
                self.pub.publish("home")                    
                    
        elif self.first_order_data in ["101_arrive", "102_arrive", "201_arrive", "202_arrive"]:
            rospy.loginfo("Received data from First_order: %s", self.first_order_data)
            self.pub.publish(self.first_order_data)
            

        # Read data from Second_order
    def cabinet_on_data_change(self, event):
        # 데이터 값이 변할 때마다 호출되는 콜백 함수
        self.cabinet()
        
    def module_on_data_change(self, event):
        self.Module()
        
    def dust_on_data_change(self, event):
        self.Dust()
    
    def order_on_data_change(self, event):
        self.drive_order()
        
    def callback_on_data_change(self, event):
        self.callback()
            
    def listener(self):
        # 파이어베이스 경로에 대한 이벤트 리스너 등록
        order_ref = db.reference('Order')  # 'Order' 경로 참조
        dust_ref = db.reference('dust')
        cabinet_ref = db.reference('Cabinet')
        module_ref = db.reference('module')  

# 특정 child 경로들에 대한 참조
        first_order_ref = order_ref.child('First_order')
        second_order_ref = order_ref.child('Second_order')
        dust_state_ref = dust_ref.child('Dust_State')
        module_state_ref = module_ref.child('Module_Mode')
        
        cabinet_ref.listen(lambda event: self.cabinet_on_data_change(event))
        first_order_ref.listen(lambda event: self.order_on_data_change(event) and self.callback_on_data_change(event))
        second_order_ref.listen(lambda event: self.order_on_data_change(event) and self.callback_on_data_change(event))
        dust_state_ref.listen(lambda event: self.dust_on_data_change(event))
        module_state_ref.listen(lambda event: self.module_on_data_change(event))

    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    credentials_path = 'test/fir-test3-62959-firebase-adminsdk-pdaz3-a3e74cabd0.json'
    database_url = 'https://fir-test3-62959-default-rtdb.firebaseio.com/'
    try:
        print(firebase_admin)
        listener = Listener()
        listener.listener()
    except rospy.ROSInterruptException:
        pass
