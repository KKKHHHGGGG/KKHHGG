import tkinter as tk
from tkinter import ttk,  IntVar, StringVar
from tkinter import font as tkFont  # 폰트 모듈 임포트
import serial
import time
from PIL import Image, ImageTk
import threading

import numpy as np
import cv2
from ultralytics import YOLO
import random
import matplotlib.pyplot as plt

class ScaraRobotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.geometry("800x600")
        self.title("SCARA Robot Control")
        self.init_yolo()
        self.wafer_state_event = threading.Event()
        

        
        # 시리얼 포트 설정. 적절한 COM 포트로 설정하세요.
        self.serial_port = serial.Serial('COM3', 115200, timeout=1)
        self.serial_port1 = serial.Serial('COM8', 115200, timeout=1) #FOUP아두이노통신
        
        # GUI 컨트롤 변수 초기화
        self.j1_slider_value = tk.DoubleVar()
        self.j2_slider_value = tk.DoubleVar()
        self.j3_slider_value = tk.DoubleVar()
        self.z_slider_value = tk.DoubleVar()
        self.speed_slider_value = tk.IntVar(value=500)
        self.acceleration_slider_value = tk.IntVar(value=500)
        # self.gripper_value = tk.IntVar(value=100)
        self.save_status = tk.IntVar(value=0)  # 추가: 저장 상태
        self.run_status = tk.IntVar(value=0)  # 추가: 실행 상태
        
        # 폰트를 불러옴 
        self.customFont = tkFont.Font(family="Arial", size=14, weight="bold")     
        self.customFont1 = tkFont.Font(family="Arial", size=10, weight="bold")     
        self.customFont2 = tkFont.Font(family="Verdana", size=35, weight="bold")     
        
           
        # 스타일 생성
        style = ttk.Style(self)
        style.configure("Exit.TButton", font=self.customFont, background='red', foreground='red')
        style.configure('Bold.TButton', font=('Helvetica', 10, 'bold'))
        style.configure('HugeBold.TButton', font=('Helvetica', 14, 'bold'))
        # 슬라이더 변수와 스텝 변수를 저장할 딕셔너리를 초기화합니다.
        self.slider_vars = {}
        
        # 위치 저장을 위한 변수
        self.positions = []
        self.positionsCounter = 0

        self.message = None
        
        self.after_id = None  # after 호출 ID를 저장할 변수
        
        self.wafer_state = None

        self.cap = None  # Initialize cap here      
        self.running = False  
        
        # 경고등을 그릴 Canvas 생성
        self.canvas = tk.Canvas(self, width=1000, height=800)
        self.canvas.pack()
        
        # 빨간색 경고등 생성
        self.FOUP1_light = self.canvas.create_oval(500, 150, 600, 250, fill='gray')
        # 초록색 경고등 생성
        self.FOUP2_light = self.canvas.create_oval(625, 150, 725, 250, fill='gray')
        # 초록색 경고등 생성
        self.Camera_light = self.canvas.create_oval(750, 150, 850, 250, fill='gray')
        # 화살표 생성: (시작 x, 시작 y, 끝 x, 끝 y)
        self.arrow_robot_wafer = self.canvas.create_line(190, 695, 300, 695, arrow=tk.LAST, fill="gray", width=13)
        self.arrow_wafer_camera = self.canvas.create_line(390, 695, 510, 695, arrow=tk.LAST, fill="gray", width=13)
        self.arrow_camera_trash = self.canvas.create_line(600, 685, 740, 645, arrow=tk.LAST, fill="gray", width=13)
        self.arrow_camera_foup = self.canvas.create_line(600, 705, 740, 735, arrow=tk.LAST, fill="gray", width=13)
        # 진행도 생성
        self.stick_1 = self.canvas.create_line(90, 790, 265, 790, fill="gray", width=13)
        self.stick_2 = self.canvas.create_line(275, 790, 445, 790, fill="gray", width=13)
        self.stick_3 = self.canvas.create_line(455, 790, 625, 790, fill="gray", width=13)
        self.stick_4 = self.canvas.create_line(635, 790, 810, 790, fill="gray", width=13)

        # GUI 컨트롤 생성
        self.create_widgets()
        
        # 저장 포지션 생성
        self.create_save_position_button()
        
        # UI 생성
        self.initialize_ui()

        # RUN/STOP 버튼 생성 및 위치 설정
        self.run_button = ttk.Button(self, text="RUN", style="HugeBold.TButton", command=self.toggleRunStatus)
        self.run_button.place(x=700, y=300, width=150, height=50)
        
        # # 텍스트 필드에 값을 입력할 때 슬라이더 값도 업데이트하기 위한 StringVar
        # self.speed_text_value = StringVar(value='500')
        # self.acceleration_text_value = StringVar(value='500')
        # # Entry 위젯과 연결된 StringVar 변수
        # self.current_speed_text_value = StringVar(value='500')
        # self.current_acceleration_text_value = StringVar(value='500')
        
        # 슬라이더 생성
        self.create_sliders()
        # home 버튼 추가
        self.create_home_button()
        # 웨이퍼 프로세싱 버튼 추가
        self.create_wafer_processing_button()
        # 종료 버튼
        self.create_Exit_button()
        # clear 버튼 추가
        self.create_Clear_button()
        
    def init_yolo(self):
        self.class_list = self.load_class_list(r"C:\Users\taery\coco.txt")
        self.detection_colors = self.assign_colors(self.class_list)
        self.model = YOLO(r"C:\Users\taery\best.pt", "v8")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("카메라 실행 불가")
            exit()
        self.running = False        
    def load_class_list(self, file_path):
        with open(file_path, 'r') as file:
            data = file.read()
            return data.split("\n")

    def assign_colors(self, class_list):
        return [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for _ in class_list]
    
    def start_detection_async(self):
        # Run detection in a separate thread to prevent UI freezing
        detection_thread = threading.Thread(target=self.start_detection)
        detection_thread.start()

    def start_detection(self):
        self.cap = cv2.VideoCapture(0)  # Initialize VideoCapture here
        if not self.cap.isOpened():
            print("Cannot open camera")
            return
        
        self.running = True
        while self.running:
            time.sleep(0.2)
            ret, frame = self.cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            detect_params = self.model.predict(source=[frame], conf=0.45, save=False)
            if len(detect_params[0]):
                self.draw_detections(frame, detect_params[0])
                classes_detected = [self.class_list[int(box.cls.numpy()[0])] for box in detect_params[0].boxes]
                if 'detective' in classes_detected:
                    self.update_message('red')
                    self.stop_detection()
                elif 'normal' in classes_detected:
                    self.update_message('green')
                    self.stop_detection()
            cv2.imshow('Wafer Detection', frame)
            self.wafer_state_event.set()

    def update_message(self, Color):
        self.canvas.itemconfig(self.Camera_light, fill=Color)
        self.wafer_state = Color
        self.wafer_state_event.set()

    def draw_detections(self, frame, detections):
        for box in detections.boxes:
            clsID = box.cls.numpy()[0]
            conf = box.conf.numpy()[0]
            bb = box.xyxy.numpy()[0]
            cv2.rectangle(frame, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), self.detection_colors[int(clsID)], 3)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, f"{self.class_list[int(clsID)]} {conf:.2f}", (int(bb[0]), int(bb[1]) - 10), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

    def stop_detection(self):
        self.running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("Detection stopped and resources released.")
        
    def initialize_ui(self):
        # Pillow를 사용하여 이미지 로드 및 크기 조정
        wafer_ui = Image.open(r'C:\Users\taery\Downloads\free-icon-waffle-10022638.png')
        resized_image = wafer_ui.resize((80, 80))  # 이미지 크기 조정
        photo = ImageTk.PhotoImage(resized_image)

        # 이미지를 표시할 라벨 생성 및 위치 지정
        label = tk.Label(self, image=photo)  # `self`를 사용
        label.place(x=300, y=655)
        label.image = photo  # 이미지 참조 유지
    
        # Pillow를 사용하여 이미지 로드 및 크기 조정
        scara_robot_ui = Image.open(r'C:\Users\taery\Downloads\free-icon-robotic-arm-1404716.png')
        resized_image = scara_robot_ui.resize((80, 80))  # 이미지 크기 조정
        photo = ImageTk.PhotoImage(resized_image)

        # 이미지를 표시할 라벨 생성 및 위치 지정
        label = tk.Label(self, image=photo)  # `self`를 사용
        label.place(x=90, y=640)
        label.image = photo  # 이미지 참조 유지
        
        
        # Pillow를 사용하여 이미지 로드 및 크기 조정
        camera_ui = Image.open(r'C:\Users\taery\Downloads\free-icon-camera-685655.png')
        resized_image = camera_ui.resize((70, 70))  # 이미지 크기 조정
        photo = ImageTk.PhotoImage(resized_image)

        # 이미지를 표시할 라벨 생성 및 위치 지정
        label = tk.Label(self, image=photo)  # `self`를 사용
        label.place(x=520, y=655)
        label.image = photo  # 이미지 참조 유지
        
        # Pillow를 사용하여 이미지 로드 및 크기 조정
        trash_ui = Image.open(r'C:\Users\taery\Downloads\free-icon-trash-can-bin-12280812.png')
        resized_image = trash_ui.resize((70, 70))  # 이미지 크기 조정
        photo = ImageTk.PhotoImage(resized_image)

        # 이미지를 표시할 라벨 생성 및 위치 지정
        label = tk.Label(self, image=photo)  # `self`를 사용
        label.place(x=745, y=610)
        label.image = photo  # 이미지 참조 유지
        
        # Pillow를 사용하여 이미지 로드 및 크기 조정
        foup_ui = Image.open(r'C:\Users\taery\Downloads\free-icon-filing-cabinet-3629552.png')
        resized_image = foup_ui.resize((60, 60))  # 이미지 크기 조정
        photo = ImageTk.PhotoImage(resized_image)

        # 이미지를 표시할 라벨 생성 및 위치 지정
        label = tk.Label(self, image=photo)  # `self`를 사용
        label.place(x=750, y=705)
        label.image = photo  # 이미지 참조 유지
        
        foup1_text = ttk.Label(self, text="FOUP1", font=self.customFont)
        foup1_text.place(x=520, y=260)
        
        foup2_text = ttk.Label(self, text="FOUP2", font=self.customFont)
        foup2_text.place(x=645, y=260)
        
        camera_text = ttk.Label(self, text="CAMERA", font=self.customFont)
        camera_text.place(x=760, y=260)
        
        MCA = ttk.Label(self, text="M C A", font=self.customFont2)
        MCA.place(x=160, y=50)
        
    def create_wafer_processing_button(self):
            # WAFER PROCESSING 버튼 생성 및 클래스 속성으로 저장
            self.wafer_processing_button = ttk.Button(self, text="WAFER PROCESSING", style="Bold.TButton", command=self.create_wafer_processing)
            self.wafer_processing_button.place(x=500, y=50, width=150, height=50)

    # 웨이퍼 이동 메커니즘 구현
    def create_wafer_processing(self):
        def process_response():
            if self.wafer_processing_button["text"] == "WAFER PROCESSING":
                # "WAFER PROCESSING" 상태일 때 동작
                self.wafer_processing_button.config(text="STOP")
                self.toggle_buttons_state('disabled')
                if self.message == 'FirstComeC':
                    print("received_FirstComeC")    
                    self.canvas.itemconfig(self.FOUP1_light, fill='green')
                    # self.canvas.itemconfig(self.FOUP2_light, fill='green')
                    self.process_steps(0)  # 시작 단계 0에서 프로세스 시작

        if self.wafer_processing_button["text"] == "WAFER PROCESSING":
            # 버튼이 "WAFER PROCESSING" 상태일 때만 FOUP 명령을 보냅니다.
            self.received_and_send_to_FOUP(1, 1, process_response)
        elif self.wafer_processing_button["text"] == "STOP":
            # "STOP" 상태일 때의 동작을 정의
            print("Stop processing")
            self.stop_processing()  # 프로세스 종료 단계 등을 처리
            self.toggle_buttons_state('normal')  # 버튼 상태를 원래대로 복구
            self.wafer_processing_button.config(text="WAFER PROCESSING")  # 버튼 텍스트를 다시 설정

    def stop_processing(self):
        # 모든 예정된 작업을 취소하고 초기화
        if self.after_id:
            self.after_cancel(self.after_id)
            self.after_id = None
        self.after_id = self.after(100, lambda: self.process_steps(12))  # 원점 복귀
        print("Processing stopped and reset to initial state.")

        
    def process_steps(self, step):
        
        if self.after_id:
            self.after_cancel(self.after_id)
            self.after_id = None
        if step == 0 and self.message == 'FirstComeC':
            time.sleep(1)
            self.set_and_send_data(20, 60, 0, -20, 0, 0, 1000, 1000)
            self.after(8000, lambda: self.process_steps(1))
        elif step == 1:
            self.set_and_send_data(70, 0, 0, -20, 0, 0, 1000, 1000)
            self.canvas.itemconfig(self.arrow_robot_wafer, fill='green')
            self.after(7000, lambda: self.process_steps(2))
        elif step == 2:
            self.set_and_send_data(70, 0, 20, 70, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(3)) 
        elif step == 3:
            self.set_and_send_data(70, 0, -20, 70, 0, 0, 500, 500)
            self.after(6000, lambda: self.process_steps(4))
        elif step == 4:
            self.set_and_send_data(70, 0, -20, -40, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(5))
        elif step == 5:
            self.set_and_send_data(24, 0, 100, 20, 0, 0, 500, 500)
            self.canvas.itemconfig(self.arrow_wafer_camera, fill='green')
            self.after(13000, lambda: self.process_steps(6))
        elif step == 6:
            self.start_detection_async()
            self.check_wafer_state(6)
            
            
        elif step == 7:
            self.canvas.itemconfig(self.Camera_light, fill='gray')
            self.set_and_send_data(24, 0, -40, 20, 0, 0, 1000, 1000)
            self.after(7000, lambda: self.process_steps(8))
        elif step == 8:
            self.received_and_send_to_FOUP(2, 0)
            self.set_and_send_data(24, -80, -60, 20, 0, 0, 500, 500)
            self.after(7000, lambda: self.process_steps(9))
        elif step == 9:
            self.canvas.itemconfig(self.FOUP2_light, fill='green')
            # self.wafer_state = None
            self.set_and_send_data(-30, -80, -60, 20, 0, 0, 500, 500)
            self.after(7000, lambda: self.process_steps(10))
        elif step == 10:
            self.set_and_send_data(-70, 0, -60, 20, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(11))
        elif step == 11:
            self.set_and_send_data(-70, 0, -20, -80, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(12))
        elif step == 12:
            self.set_and_send_data(-70, 0, 20, -80, 0, 0, 500, 500)
            self.canvas.itemconfig(self.arrow_camera_foup, fill='green')
            self.after(13000, lambda: self.process_steps(13))
        elif step == 13:
            self.set_and_send_data(-70, 0, 20, 40, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(14))
        elif step == 14:
            self.set_and_send_data(-70, 0, -60, 40, 0, 0, 500, 500)
            self.after(9000, lambda: self.process_steps(15))
        elif step == 15:
            self.set_and_send_data(-40, 50, -60, 30, 0, 0, 500, 500)
            self.after(8000, lambda: self.process_steps(16))
        elif step == 16:
            self.set_and_send_data(30, 60, -60, -30, 0, 0, 500, 500)
            self.canvas.itemconfig(self.stick_1, fill='green')
            self.canvas.itemconfig(self.arrow_camera_foup, fill='gray')
            self.canvas.itemconfig(self.arrow_wafer_camera, fill='gray')
            self.canvas.itemconfig(self.arrow_robot_wafer, fill='gray')
            self.canvas.itemconfig(self.arrow_camera_trash, fill='gray')
            self.after(10000, lambda: self.process_steps(20))
            
            
        elif step == 106:
            self.canvas.itemconfig(self.Camera_light, fill='gray')
            self.set_and_send_data(24, 0, -40, 20, 0, 0, 1000, 1000)
            self.after(10000, lambda: self.process_steps(107))
        elif step == 107:
            self.set_and_send_data(-55, 80, -40, -90, 0, 0, 500, 500)
            self.after(16000, lambda: self.process_steps(108))
        elif step == 108:
            self.set_and_send_data(-55, 80, 35, -90, 0, 0, 500, 500)
            self.after(8000, lambda: self.process_steps(109))
        elif step == 109:
            self.set_and_send_data(-55, 80, 35, 10, 0, 0, 500, 500)
            self.canvas.itemconfig(self.arrow_camera_trash, fill='green')
            self.after(13000, lambda: self.process_steps(110))
        elif step == 110:
            self.set_and_send_data(-55, 80, 35, 10, 0, 0, 500, 500)
            self.after(8000, lambda: self.process_steps(16))            
            
            
        elif step == 20:
            self.set_and_send_data(30, 60, 0, -30, 0, 0, 1000, 1000)
            self.canvas.itemconfig(self.arrow_robot_wafer, fill='green')
            self.after(7000, lambda: self.process_steps(21))
        elif step == 21:
            self.set_and_send_data(70, 0, 60, -30, 0, 0, 1000, 1000)
            self.after(9000, lambda: self.process_steps(22)) 
        elif step == 22:
            self.set_and_send_data(70, 0, 60, 70, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(23)) 
        elif step == 23:
            self.set_and_send_data(70, 0, 20, 70, 0, 0, 500, 500)
            self.after(6000, lambda: self.process_steps(24))
        elif step == 24:
            self.set_and_send_data(70, 0, 20, -40, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(25))
        elif step == 25:
            self.set_and_send_data(24, 0, 100, 20, 0, 0, 500, 500)
            # self.canvas.itemconfig(self.FOUP1_light, fill='gray')
            self.canvas.itemconfig(self.arrow_wafer_camera, fill='green')
            self.after(15000, lambda: self.process_steps(26))
        elif step == 26:
            self.start_detection_async()
            self.check_wafer_state(26)
            
            
        elif step == 27:
            
            self.canvas.itemconfig(self.Camera_light, fill='gray')           
            self.set_and_send_data(24, 0, -60, 20, 0, 0, 1000, 1000)
            self.after(7000, lambda: self.process_steps(28))
        elif step == 28:
            self.wafer_state = None
            self.set_and_send_data(24, -80, -60, 20, 0, 0, 500, 500)
            self.after(7000, lambda: self.process_steps(29))
        elif step == 29:
            self.wafer_state = None
            self.set_and_send_data(10, -80, -60, 20, 0, 0, 500, 500)
            self.after(7000, lambda: self.process_steps(30))
        elif step == 30:
            self.set_and_send_data(-70, 0, -60, 20, 0, 0, 500, 500)
            self.after(13000, lambda: self.process_steps(31))
        elif step == 31:
            self.set_and_send_data(-70, 0, 20, -80, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(32))
        elif step == 32:
            self.set_and_send_data(-70, 0, 60, -80, 0, 0, 300, 300)
            self.canvas.itemconfig(self.arrow_camera_foup, fill='green')
            self.after(13000, lambda: self.process_steps(33))
        elif step == 33:
            self.set_and_send_data(-70, 0, 60, 40, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(34))
        elif step == 34:
            self.set_and_send_data(-70, 0, -60, 40, 0, 0, 500, 500)
            self.after(6000, lambda: self.process_steps(35))
        elif step == 35:
            self.set_and_send_data(-40, 50, -60, 30, 0, 0, 500, 500)
            self.after(8000, lambda: self.process_steps(36))
        elif step == 36:
            self.set_and_send_data(30, 60, 0, -30, 0, 0, 1000, 1000)
            self.canvas.itemconfig(self.stick_2, fill='green')
            self.canvas.itemconfig(self.arrow_camera_foup, fill='gray')
            self.canvas.itemconfig(self.arrow_wafer_camera, fill='gray')
            self.canvas.itemconfig(self.arrow_robot_wafer, fill='gray')
            self.canvas.itemconfig(self.arrow_camera_trash, fill='gray')
            self.after(9000, lambda: self.process_steps(41))
            
            
        elif step == 126:
            self.canvas.itemconfig(self.Camera_light, fill='gray')
            self.set_and_send_data(-55, 80, 35, -90, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(129))
        elif step == 127:
            self.set_and_send_data(-55, 80, 35, -90, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(128))
        elif step == 128:
            self.set_and_send_data(-55, 80, 80, -90, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(129))
        elif step == 129:
            self.set_and_send_data(-55, 80, 80, 10, 0, 0, 500, 500)
            self.canvas.itemconfig(self.arrow_camera_trash, fill='green')
            self.after(10000, lambda: self.process_steps(36))

        elif step == 40:
            self.set_and_send_data(30, 60, 0, -30, 0, 0, 1000, 1000)
            self.canvas.itemconfig(self.arrow_robot_wafer, fill='green')
            self.after(7000, lambda: self.process_steps(41))
        elif step == 41:
            self.set_and_send_data(70, 0, 110, -30, 0, 0, 500, 500)
            self.canvas.itemconfig(self.arrow_robot_wafer, fill='green')
            self.after(9000, lambda: self.process_steps(42)) 
        elif step == 42:
            self.set_and_send_data(70, 0, 110, 70, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(43)) 
        elif step == 43:
            self.set_and_send_data(70, 0, 70, 70, 0, 0, 500, 500)
            self.after(6000, lambda: self.process_steps(44))
        elif step == 44:
            self.set_and_send_data(70, 0, 70, -30, 0, 0, 1000, 1000)
            self.after(15000, lambda: self.process_steps(45))
        elif step == 45:
            self.set_and_send_data(24, 0, 100, 20, 0, 0, 500, 500)
            self.canvas.itemconfig(self.FOUP1_light, fill='gray')
            self.canvas.itemconfig(self.arrow_wafer_camera, fill='green')
            self.after(13000, lambda: self.process_steps(46))
        elif step == 46:
            self.start_detection_async()
            self.check_wafer_state(46)

        elif step == 47:
            self.canvas.itemconfig(self.Camera_light, fill='gray')
            self.set_and_send_data(24, 0, -40, 20, 0, 0, 1000, 1000)
            self.after(7000, lambda: self.process_steps(48))
        elif step == 48:
            self.received_and_send_to_FOUP(1, 0) 
            self.set_and_send_data(24, -80, -60, 20, 0, 0, 500, 500)
            self.after(7000, lambda: self.process_steps(49))
        elif step == 49:
            self.canvas.itemconfig(self.FOUP1_light, fill='gray')
            # self.wafer_state = None
            self.set_and_send_data(-30, -80, -60, 20, 0, 0, 500, 500)
            self.after(7000, lambda: self.process_steps(50))
        elif step == 50:
            self.set_and_send_data(-70, 0, -60, 20, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(51))
        elif step == 51:
            self.set_and_send_data(-70, 0, 70, -80, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(52))
        elif step == 52:
            self.set_and_send_data(-70, 0, 115, -80, 0, 0, 500, 500)
            self.canvas.itemconfig(self.arrow_camera_foup, fill='green')
            self.after(13000, lambda: self.process_steps(53))
        elif step == 53:
            self.set_and_send_data(-70, 0, 115, 40, 0, 0, 1000, 1000)
            self.after(13000, lambda: self.process_steps(54))
        elif step == 54:
            self.set_and_send_data(-70, 0, -60, 40, 0, 0, 1000, 1000)
            self.after(9000, lambda: self.process_steps(55))
        elif step == 55:
            self.set_and_send_data(-40, 50, -60, 30, 0, 0, 500, 500)
            self.after(8000, lambda: self.process_steps(56))
        elif step == 56:
            self.set_and_send_data(0, 0, 0, 0, 0, 0, 500, 500)
            self.canvas.itemconfig(self.stick_3, fill='green')
            self.canvas.itemconfig(self.arrow_camera_foup, fill='gray')
            self.canvas.itemconfig(self.arrow_wafer_camera, fill='gray')
            self.canvas.itemconfig(self.arrow_robot_wafer, fill='gray')
            self.canvas.itemconfig(self.arrow_camera_trash, fill='gray')
            self.received_and_send_to_FOUP(2, 1)
            self.canvas.itemconfig(self.FOUP2_light, fill='gray')
            self.after(10000, lambda: self.process_steps(60))

        elif step == 146:
            self.canvas.itemconfig(self.Camera_light, fill='gray')
            self.received_and_send_to_FOUP(1, 0) 
            self.set_and_send_data(-55, 80, 85, -90, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(148))
        elif step == 147:
            self.set_and_send_data(-55, 80, 85, -90, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(148))
        elif step == 148:
            self.set_and_send_data(-55, 80, 120, -90, 0, 0, 500, 500)
            self.after(10000, lambda: self.process_steps(149))
        elif step == 149:
            self.set_and_send_data(-55, 80, 120, 10, 0, 0, 500, 500)
            
            self.canvas.itemconfig(self.arrow_camera_trash, fill='green')
            self.after(10000, lambda: self.process_steps(56))

            
    def check_wafer_state(self, step):
        if self.wafer_state_event.is_set():
            self.wafer_state_event.clear()
            if self.wafer_state == 'red':
                print("detected")
                self.wafer_state = None
                self.after_id = self.after(1000, lambda: self.process_steps(step+100))
                
            elif self.wafer_state == 'green':
                print("normal")
                self.wafer_state = None
                self.after_id = self.after(1000, lambda: self.process_steps(step+1))
        else:
            self.after_id = self.after(100, lambda: self.check_wafer_state(step))


    def toggle_buttons_state(self, state):
        # save_position_button, home_button, run_button의 상태를 변경
        self.save_position_button.config(state=state)
        self.home_button.config(state=state)
        self.run_button.config(state=state)
        self.Clear_button.config(state=state)
        self.minus_button.config(state=state)
        self.plus_button.config(state=state)

                    
    def create_widgets(self):
        self.create_slider_with_buttons("J1", -100, 100, 100, 180)
        self.create_slider_with_buttons("J2", -100, 100, 100, 290)
        self.create_slider_with_buttons("J3", -100, 100, 100, 400)
        self.create_slider_with_buttons("Z", -100, 100, 100, 500)

        # 여기에 J2, J3, Z에 대한 비슷한 코드를 추가할 수 있습니다.
        
    def create_slider_with_buttons(self, label, min_val, max_val, x, y):
        # 라벨 위치 설정
        ttk.Label(self, text=f"{label}", font=self.customFont).place(x=x-50, y=y)

        # 슬라이더 변수 생성 및 저장
        slider_var = getattr(self, f"{label.lower()}_slider_value")  # 시작 값이 0이라고 가정
        self.slider_vars[label] = slider_var

        # 슬라이더 위치 설정
        slider = ttk.Scale(self, from_=min_val, to=max_val, orient='horizontal', variable=slider_var)
        slider.place(x=x, y=y, width=270)

        # "+" 및 "-" 버튼 작동량을 설정하는 변수. 각 슬라이더별로 별도 설정.
        step_var = tk.StringVar(value="1")
        self.slider_vars[f'{label}_step'] = step_var

        # 슬라이더 값 입력 필드 (현재 값 표시용)
        entry = ttk.Entry(self, textvariable=slider_var, width=5)
        entry.place(x=x+320, y=y+5)

        # 슬라이더 이동량 설정 입력 필드
        step_entry = ttk.Entry(self, textvariable=step_var, width=5)
        step_entry.place(x=x+110, y=y+40)

        # "-" 버튼 위치 설정
        self.minus_button = ttk.Button(self, text="-", style="HugeBold.TButton", command=lambda: self.adjust_slider(slider_var, step_var.get(), -1))
        self.minus_button.place(x=x, y=y+40, width=50, height=25)
        # "+" 버튼 위치 설정
        self.plus_button = ttk.Button(self, text="+", style="HugeBold.TButton", command=lambda: self.adjust_slider(slider_var, step_var.get(), 1))
        self.plus_button.place(x=x+245, y=y+40, width=50, height=25)
        
    def create_save_position_button(self):
        # SAVE POSITION 버튼 생성 및 클래스 속성으로 저장
        self.save_position_button = ttk.Button(self, text="SAVE", style="HugeBold.TButton", command=self.savePosition)
        self.save_position_button.place(x=500, y=300, width=150, height=50)

    # def create_run_button(self):
    #     # RUN 버튼 생성 및 위치 설정, 폰트 적용
    #     ttk.Button(self, text="RUN", command=self.toggleRunStatus).place(x=700, y=300, width=150, height=50)
    
    def create_sliders(self):
        # 속도 슬라이더 생성
        ttk.Label(self, text="SPEED", font=self.customFont1).place(x=550, y=480)
        ttk.Scale(self, from_=0, to=1000, orient="horizontal", variable=self.speed_slider_value, command=lambda event=None: self.update_and_send_data(), length=150).place(x=500, y=500)
        speed_entry = ttk.Entry(self, textvariable=self.speed_slider_value, width=3)
        speed_entry.place(x=560, y=530)

        # 가속도 슬라이더 생성
        ttk.Label(self, text="ACCELETION", font=self.customFont1).place(x=735, y=480)
        ttk.Scale(self, from_=0, to=1000, orient="horizontal", variable=self.acceleration_slider_value, command=lambda event=None: self.update_and_send_data(), length=150).place(x=700, y=500)
        acceleration_entry = ttk.Entry(self, textvariable=self.acceleration_slider_value, width=3)
        acceleration_entry.place(x=760, y=530)
        
    # def validate_int(self, speed_text_value, acceleration_text_value):
    #     # 현재 텍스트 값을 가져옴
    #     current_speed_text_value = speed_text_value.get()
    #     current_acceleration_text_valuee = acceleration_text_value.get()
    #     # 값이 정수로 변환 가능한지 확인
    #     try:
    #         int(current_speed_text_value)  # 시도해보고 변환 가능하면 문제 없음
    #         int(current_acceleration_text_valuee)
    #     except ValueError:
    #         # 변환 불가능하면, 마지막 유효한 값으로 되돌림
    #         speed_text_value.set(self.speed_slider_value.get())
    #         acceleration_text_value.set(self.acceleration_slider_value.get())

    def toggleRunStatus(self):
        if self.run_status.get() == 0:
            self.run_status.set(1)
            self.run_button.config(text="STOP")
        else:
            # self.run_status.set(0)
            self.run_button.config(text="RUN")

        # 변경된 run_status 값으로 데이터 전송
        self.update_and_send_data()

        # 테스트 출력
        print(f"Run status toggled to {self.run_status.get()}")

    def adjust_slider(self, slider_var, step, direction):
        try:
            step = float(step)  # step가 문자열이거나 숫자라고 가정하고 float으로 변환합니다.
        except ValueError:
            step = 1.0  # 변환에 실패할 경우 기본값 1.0을 사용합니다.

        current_value = slider_var.get() + (step * direction)
        slider_var.set(current_value)   
        self.update_and_send_data()
    
    def create_home_button(self):
        self.home_button = ttk.Button(self, text="HOME", style="HugeBold.TButton", command=self.send_home_command)
        self.home_button.place(x=500, y=420, width=350, height=50)

    def create_Clear_button(self):
        self.Clear_button = ttk.Button(self, text="CLEAR", style="HugeBold.TButton", command=self.ClearPosition)
        self.Clear_button.place(x=500, y=360, width=350, height=50)

    def create_Exit_button(self):
        # Close 버튼 생성 및 위치 설정
        self.Exit_button = ttk.Button(self, text="EXIT", style="Exit.TButton",command=self.send_home_and_close)
        self.Exit_button.place(x=790, y=50, width=60, height=60)

    def send_home_and_close(self):
        # Home 명령을 보내는 함수 호출
        self.send_home_command()
        # 버튼 비활성화
        self.Exit_button.config(state='disabled')
        # 2초 후 애플리케이션 종료
        self.after(2000, self.destroy)

        
    def send_home_command(self):
    # 각 슬라이더의 값을 설정
        self.j1_slider_value.set(0)
        self.j2_slider_value.set(0)
        self.j3_slider_value.set(0)
        self.z_slider_value.set(0)
        # 저장 상태와 실행 상태는 0으로 초기화
        self.save_status.set(0)
        self.run_status.set(0)
        # 속도와 가속도를 500으로 설정
        self.speed_slider_value.set(500)
        self.acceleration_slider_value.set(500)
        self.update_and_send_data()
        
    def set_and_send_data(self, j1_val, j2_val, j3_val, z_val, save_status_val, run_status_val, speed_val, acceleration_val):
        # 각 슬라이더의 값을 설정
        self.j1_slider_value.set(j1_val)
        self.j2_slider_value.set(j2_val)
        self.j3_slider_value.set(j3_val)
        self.z_slider_value.set(z_val)
        # 저장 상태와 실행 상태 설정
        self.save_status.set(save_status_val)
        self.run_status.set(run_status_val)
        # 속도와 가속도를 설정
        self.speed_slider_value.set(speed_val)
        self.acceleration_slider_value.set(acceleration_val)
        # 변경된 데이터 전송
        self.update_and_send_data()
        
    def ClearPosition(self):
        self.save_status.set(2)
        self.update_and_send_data()
        print("Saved Position Clear")
        self.save_status.set(0)
        
    def savePosition(self):
        # save_status 값을 1로 설정
        self.save_status.set(1)
        
        # 변경된 save_status 값으로 데이터 전송
        self.update_and_send_data()
        
        # Position saved 메시지 출력
        print("Position saved")
        
        # 데이터 전송 후 save_status 값을 다시 0으로 초기화
        self.save_status.set(0)


    def update_and_send_data(self):
        # 데이터 문자열 생성. 데이터 필드의 의미를 정확하게 반영합니다.
        data_str = "{},{},{},{},{},{},{},{}".format(
            self.save_status.get(), self.run_status.get(),
            int(self.j1_slider_value.get()), int(self.j2_slider_value.get()),
            int(self.j3_slider_value.get()), int(self.z_slider_value.get()),
            int(self.speed_slider_value.get()), int(self.acceleration_slider_value.get())
        )
        
        # 데이터 전송
        print(f"Sending: {data_str}")
        self.serial_port.write(data_str.encode() + b'\n')  # 개행 문자 추가
    
 # 풉 아두이노와의 통신
    def received_and_send_to_FOUP(self, FOUP_num, send_data, callback=None):
        # 먼저 아두이노에 명령을 전송
        if FOUP_num == 1:
            if send_data == 0:
                command = "FirstGo"
            elif send_data == 1:
                command = "FirstCome"
        elif FOUP_num == 2:
            if send_data == 0:
                command = "SecondCome"
            elif send_data == 1:
                command = "SecondGo"

        print(f"Sending command: {command}")
        self.serial_port1.write(command.encode())
        self.wait_for_response(callback)
        
    def wait_for_response(self, callback=None):
        # 대기 상태가 아닌 경우 대기
        while self.serial_port1.in_waiting == 0:
            time.sleep(0.1)  # CPU 사용을 줄이기 위해 적당한 지연 시간 설정

        response = self.serial_port1.readline().decode().strip()
        print(f"Received: '{response}'")
        self.message = response
        if callback:
            callback()
          
if __name__ == "__main__":
    # plt.ioff()
    app = ScaraRobotGUI()
    app.mainloop()
