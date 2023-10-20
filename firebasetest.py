#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 모듈
import rospy
from std_msgs.msg import String
import signal
import sys
import time

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
        self.pub = rospy.Publisher("firebase_to_ros", String, queue_size=1000)
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
        Order_reb = db.reference('Order')
        firstorder_ref = Order_reb.child('First_order')
        secondorder_ref = Order_reb.child('Second_order')
        # 호텔 경로
        if message.data in ["Hotel_Mode", "Other_Mode", "None"]:
            if message.data in ["Hotel_Mode", "Other_Mode"]:
                
                module_data = {"Module_Mode": message.data }
                ref.child('module').set(module_data)
            elif message.data == "None":
                
                module_data = {"Module_Mode":"None"}
                ref.child('module').set(module_data) 
        # 미세먼지 경로
        elif message.data in ["GOOD", "NORMAL", "BAD", "VERY BAD"]:
            dust_data = {"dust": message.data}
            ref.update(dust_data)
        # 서랍 경로 및 주행 상호작용
        elif message.data in ["Unlock1_done", "Unlock2_done", "Lock1_done", "Lock2_done"]:
            cabinet_data = {"Cabinet": message.data}
            ref.update(cabinet_data)
            if message.data in ["Lock1_done", "Lock2_done"]:
                next = "Next"
                first_data = firstorder_ref.get() # 변수에 데이터 저장 
                second_data = secondorder_ref.get() # 변수에 데이터 저장 
                # order경로의 두 번째 인덱스 추출
                first_order_state = first_data.split('_')[1] if len(first_data.split('_')) > 1 else None
                second_order_state = second_data.split('_')[1] if len(second_data.split('_')) > 1 else None
                if first_order_state == "arrive":
                    time.sleep(0.1)
                    Order_reb.child('First_order').set(next)
                    cabinet_data = {"Cabinet": "STANBY"}
                    ref.update(cabinet_data)
                if second_order_state == "arrive":
                    time.sleep(0.1)
                    Order_reb.child('Second_order').set(next)
                    cabinet_data = {"Cabinet": "STANBY"}
                    ref.update(cabinet_data)

        
        #호수 위치 도착            
        elif message.data in ["101_arrive", "102_arrive", "201_arrive", "202_arrive"]:
            
            number_cur = int(message.data.split('_')[0])# 첫 번째 인덱스 추출
            first_data = firstorder_ref.get() # 변수에 데이터 저장 
            second_data = secondorder_ref.get() # 변수에 데이터 저장 
            self.pub.publish("drive_done")

            if first_data in ["101_go", "102_go", "201_go", "202_go"]:
                first_number_prev = int(first_data.split('_')[0])
                if number_cur == first_number_prev:
                    
                    firstorder_ref.set(message.data)
                    
            if second_data in ["101_go", "102_go", "201_go", "202_go"]:
                second_number_prev = int(second_data.split('_')[0])
                if number_cur == second_number_prev:
                    
                    secondorder_ref.set(message.data)
        
        # home도착시
        elif message.data == "home_ARRIVE":
            time.sleep(1)
            firstorder_ref.set(message.data)
            secondorder_ref.set(message.data)
            cabinet_data = {"Cabinet": "STANBY"}
            ref.update(cabinet_data)
            
        
        else:
            data = {"Order": message.data}
            rospy.loginfo("Received data from Firebase: %s", data)
            ref.child('Order').child('hihi').set(message.data)


    # 서랍 경로 데이터 확인 및 publish
    def cabinet(self):
        ref = db.reference('')
        cabinet = db.reference('Cabinet')
        self.cabinet_data = cabinet.get()  # 변수에 데이터 저장     
        if self.cabinet_data in ["First_Open", "Second_Open", "STANBY"]:
            rospy.loginfo("Received data from cabinet: %s", self.cabinet_data)
            self.pub.publish(self.cabinet_data)
        if self.cabinet_data in ["Unlock1_done", "Unlock2_done"]:
            start_time = time.time()
        
            while time.time() - start_time < 5:
                new_data = cabinet.get()  # Firebase에서 새로운 데이터 가져오기
                if new_data in ["First_Close", "Second_Close"]:
                    time.sleep(0.1)
                    self.pub.publish(new_data)
                    break  # 10초 안에 원하는 데이터가 들어오면 반복문 종료
            if time.time() - start_time > 5:
                if self.cabinet_data == "Unlock1_done":
                    time.sleep(0.1)
                    self.pub.publish("First_Close")
                    cabinet_data = {"Cabinet": "First_Close"}
                    ref.update(cabinet_data)
                    
                if self.cabinet_data == "Unlock2_done":
                    time.sleep(0.1)
                    self.pub.publish("Second_Close")
                    cabinet_data = {"Cabinet": "Second_Close"}
                    ref.update(cabinet_data)
                    
    # 모듈 경로 데이터 확인 및 publish        
    def Module(self):
        module = db.reference('module')
        self.module_mode = module.child('Module_Mode').get() # 변수에 데이터 저장 
        if self.module_mode in ["Hotel_Mode", "Open"]:
            rospy.loginfo("Received data from Module: %s", self.module_mode)
            self.pub.publish(self.module_mode)    
    
    # 미세먼지 경로 데이터 확인 및 publish        
    def Dust(self):
        Dust = db.reference('dust')
        self.Dust_data = Dust.get()  # 변수에 데이터 저장     
        if self.Dust_data in ["Dust_ON", "Dust_OFF"]:
            rospy.loginfo("Received data from Dust: %s", self.Dust_data)
            self.pub.publish(self.Dust_data)
    
    # 서랍 경로 데이터 확인 및 publish
    def drive_order(self):
        order = db.reference('Order')  
           
        self.first_order_data = order.child('First_order').get()       
        self.second_order_data = None       
         
        # first_order우선 publish        
        if self.first_order_data in ["101_go", "102_go", "201_go", "202_go", "Next"]:
            rospy.loginfo("Received data from First_order: %s", self.first_order_data)
            if self.first_order_data in ["101_go", "102_go", "201_go", "202_go"]:
                time.sleep(1)
                self.pub.publish(self.first_order_data)

            
            elif self.first_order_data == "Next":
                
                self.second_order_data = order.child('Second_order').get() 
                if self.second_order_data in ["101_go", "102_go", "201_go", "202_go"]:
                    rospy.loginfo("Received data from Second_order: %s", self.second_order_data)
                    time.sleep(1)
                    self.pub.publish(self.second_order_data)
                elif self.second_order_data in ["101_arrive", "102_arrive", "201_arrive", "202_arrive"]:
                    rospy.loginfo("Received data from Second_order: %s", self.second_order_data)
                    time.sleep(1)
                    self.pub.publish(self.second_order_data)
                    
            if self.first_order_data == "Next" and (self.second_order_data == "Next" or self.second_order_data == "home_ARRIVE"):
                rospy.loginfo("home")
                self.pub.publish("home")                    
                    
        elif self.first_order_data in ["101_arrive", "102_arrive", "201_arrive", "202_arrive"]:
            rospy.loginfo("Received data from First_order: %s", self.first_order_data)
            self.pub.publish(self.first_order_data)

        elif self.first_order_data == "home_ARRIVE":
            rospy.loginfo("Received data from First_order: %s", self.first_order_data)
            self.pub.publish(self.first_order_data)
            
    # 데이터 값이 변할 때마다 호출되는 콜백 함수
    def cabinet_on_data_change(self, event):
        
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
        dust_ref = db.reference('dust') # 'dust' 경로 참조
        cabinet_ref = db.reference('Cabinet')   # 'Cabinet' 경로 참조
        module_ref = db.reference('module')  # 'module' 경로 참조

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

