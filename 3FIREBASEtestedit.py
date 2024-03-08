#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 모듈
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
import signal
import sys
import time
import math

# 파이어베이스
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from collections import deque

# 종료문구
def signal_handler(sig, frame):
    print("Exiting...")
    sys.exit(0)
    
# 파이어베이스 설정
class Listener:
    

    
    def __init__(self):
        # ros토픽 생성
        rospy.init_node('rosFirebase', anonymous=True)
        rospy.Subscriber("ros_to_firebase", String, callback=self.callback)
        self.pub = rospy.Publisher("firebase_to_ros", String and Int32, queue_size=1000)
        # 파이어베이스 j.son경로 및 url
        self.credentials_path = credentials_path
        self.database_url = database_url
        self.data_queue = deque()
        
        # Certification 키
        self.cred = credentials.Certificate(credentials_path)

        firebase_admin.initialize_app(self.cred, {
            'databaseURL': database_url  # Database url
        })
        self.hehe = None  # 변수 초기화
        self.first_order_received = True  # 변수 초기화
    
    # subscriber 및 파이어베이스 업데이트    
    def callback(self, message):
        # 경로 참조
        ref = db.reference('')
 


    # 서랍 경로 데이터 확인 및 publish
    def cabinet(self):
        ref = db.reference('')


                    
    # 모듈 경로 데이터 확인 및 publish        
    def Module(self):
        module = db.reference('module')
   
    
    # 미세먼지 경로 데이터 확인 및 publish        
    def Dust(self):
        Dust = db.reference('dust')

    
    # jog운동
    def jog(self):
        ref = db.reference('')
        jog_ref = db.reference('jog')
        self.jog1_data = jog_ref.child('jog1').get() 
        self.jog2_data = jog_ref.child('jog2').get() 
        self.jog3_data = jog_ref.child('jog3').get() 
        self.z_data = jog_ref.child('z').get() 
        
        if self.jog1_data == "Stop" or (isinstance(self.jog1_data, int) and -100 <= self.jog1_data <= 100):
            rospy.loginfo("Received data from jog1: %s", self.jog1_data)
            if isinstance(self.jog1_data, int) and -100 <= self.jog1_data <= 100:
                time.sleep(0.1)
                self.pub.publish(self.jog1_data)
            elif self.jog1_data == "Stop":
                rospy.loginfo("Received data from jog1: %s", self.jog1_data)
                time.sleep(0.1)
                self.pub.publish("Stop")
                
        if self.jog2_data == "Stop" or (isinstance(self.jog2_data, int) and -100 <= self.jog2_data <= 100):
            rospy.loginfo("Received data from jog2: %s", self.jog2_data)
            if isinstance(self.jog2_data, int) and -100 <= self.jog2_data <= 100:
                time.sleep(0.1)
                self.pub.publish(self.jog2_data)
            elif self.jog2_data == "Stop":
                rospy.loginfo("Received data from jog2: %s", self.jog2_data)
                time.sleep(0.1)
                self.pub.publish("Stop")

        if self.jog3_data == "Stop" or (isinstance(self.jog3_data, int) and -100 <= self.jog3_data <= 100):
            rospy.loginfo("Received data from jog3: %s", self.jog3_data)
            if isinstance(self.jog3_data, int) and -100 <= self.jog3_data <= 100:
                time.sleep(0.1)
                self.pub.publish(self.jog3_data)
            elif self.jog3_data == "Stop":
                rospy.loginfo("Received data from jog3: %s", self.jog3_data)
                time.sleep(0.1)
                self.pub.publish("Stop")

        if self.z_data == "Stop" or (isinstance(self.z_data, int) and -100 <= self.z_data <= 100):
            rospy.loginfo("Received data from z: %s", self.z_data)
            if isinstance(self.z_data, int) and -100 <= self.z_data <= 100:
                time.sleep(0.1)
                self.pub.publish(self.z_data)
            elif self.z_data == "Stop":
                rospy.loginfo("Received data from z: %s", self.z_data)
                time.sleep(0.1)
                self.pub.publish("Stop")


    # 데이터 값이 변할 때마다 호출되는 콜백 함수
    def cabinet_on_data_change(self, event):
        
        self.cabinet()
        
    def module_on_data_change(self, event):
        self.Module()
        
    def dust_on_data_change(self, event):
        self.Dust()
    
    def jog_on_data_change(self, event):
        self.jog()
        
    def callback_on_data_change(self, event):
        self.callback()
            
    def listener(self):
        # 파이어베이스 경로에 대한 이벤트 리스너 등록
        jog_ref = db.reference('jog')  # 'Order' 경로 참조
        dust_ref = db.reference('dust') # 'dust' 경로 참조
        cabinet_ref = db.reference('Cabinet')   # 'Cabinet' 경로 참조
        module_ref = db.reference('module')  # 'module' 경로 참조

# 특정 child 경로들에 대한 참조
        jog1_ref = jog_ref.child('jog1')
        jog2_ref = jog_ref.child('jog2')
        jog3_ref = jog_ref.child('jog3')
        z_ref = jog_ref.child('z')
        dust_state_ref = dust_ref.child('Dust_State')
        module_state_ref = module_ref.child('Module_Mode')
        
        cabinet_ref.listen(lambda event: self.cabinet_on_data_change(event))
        jog1_ref.listen(lambda event: self.jog_on_data_change(event) and self.callback_on_data_change(event))
        jog2_ref.listen(lambda event: self.jog_on_data_change(event) and self.callback_on_data_change(event))
        jog3_ref.listen(lambda event: self.jog_on_data_change(event) and self.callback_on_data_change(event))
        z_ref.listen(lambda event: self.jog_on_data_change(event) and self.callback_on_data_change(event))
        dust_state_ref.listen(lambda event: self.dust_on_data_change(event))
        module_state_ref.listen(lambda event: self.module_on_data_change(event))

    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    #경로 지정
    credentials_path = 'mca-test-e270e-firebase-adminsdk-wthsv-e5ef79fec3.json'
    database_url = 'https://mca-test-e270e-default-rtdb.firebaseio.com/'
    try:
        #firebase라이브러리 경로 출력
        print(firebase_admin)
        listener = Listener()
        listener.listener()
    except rospy.ROSInterruptException:
        pass
