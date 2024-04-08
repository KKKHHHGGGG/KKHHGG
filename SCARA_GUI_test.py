import tkinter as tk
from tkinter import ttk,  IntVar, StringVar
from tkinter import font as tkFont  # 폰트 모듈 임포트
import serial
import time
from PIL import Image, ImageTk

class ScaraRobotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.geometry("800x600")
        self.title("SCARA Robot Control")
        
        # 시리얼 포트 설정. 적절한 COM 포트로 설정하세요.
        self.serial_port = serial.Serial('COM3', 115200, timeout=1)
        # self.serial_port1 = serial.Serial('COM6', 115200, timeout=1) #FOUP아두이노통신
        
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
        
        
        # 슬라이더 변수와 스텝 변수를 저장할 딕셔너리를 초기화합니다.
        self.slider_vars = {}
        
        # 위치 저장을 위한 변수
        self.positions = []
        self.positionsCounter = 0

        self.message = 0
        
        # 폰트를 불러옴
        self.customFont = tkFont.Font(family="Arial", size=14, weight="bold")
        
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
        self.run_button = ttk.Button(self, text="RUN", command=self.toggleRunStatus)
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
        
    def create_wafer_processing_button(self):
            # WAFER PROCESSING 버튼 생성 및 클래스 속성으로 저장
            self.wafer_processing_button = ttk.Button(self, text="WAFER PROCESSING", command=self.create_wafer_processing)
            self.wafer_processing_button.place(x=500, y=50, width=150, height=50)

    # 웨이퍼 이동 메커니즘 구현
    def create_wafer_processing(self):
        if self.wafer_processing_button["text"] == "WAFER PROCESSING":
            self.send_home_command()
            # 버튼의 텍스트를 STOP으로 변경
            self.wafer_processing_button.config(text="STOP")
            self.toggle_buttons_state('disabled')

            # FUOP 도착시 이동
            # if self.message == 1_1:
            #    self.set_and_send_data(50, 30, 40, 10, 500, 500)
            #    time.sleep(5)
            #    self.set_and_send_data(50, 30, 40, 10, 500, 500)
            #    time.sleep(5)
            #    self.set_and_send_data(50, 30, 40, 10, 500, 500)
            #    time.sleep(5)
            #    self.received_and_send_to_FOUP(1, 0) #첫번째 풉에 있는 웨이퍼를 다 뺐다는 표시
               
        elif self.wafer_processing_button["text"] == "STOP":
            # STOP 버튼을 누른 경우의 동작 정의
            # 예를 들어, 아두이노에 STOP 명령어를 보내거나, 다른 초기화 작업을 수행할 수 있습니다.
            print("Stop processing")
            self.run_status.set(0)
            self.update_and_send_data()
            self.toggle_buttons_state('normal')
            # 필요에 따라 다른 초기화 작업을 여기에 추가
            self.wafer_processing_button.config(text="WAFER PROCESSING") 

    def toggle_buttons_state(self, state):
        # save_position_button, home_button, run_button의 상태를 변경
        self.save_position_button.config(state=state)
        self.home_button.config(state=state)
        self.run_button.config(state=state)

                    
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
        ttk.Button(self, text="-", command=lambda: self.adjust_slider(slider_var, step_var.get(), -1)).place(x=x, y=y+40, width=50, height=25)

        # "+" 버튼 위치 설정
        ttk.Button(self, text="+", command=lambda: self.adjust_slider(slider_var, step_var.get(), 1)).place(x=x+245, y=y+40, width=50, height=25)

    def create_save_position_button(self):
        # SAVE POSITION 버튼 생성 및 클래스 속성으로 저장
        self.save_position_button = ttk.Button(self, text="SAVE POSITION", command=self.savePosition)
        self.save_position_button.place(x=500, y=300, width=150, height=50)

    # def create_run_button(self):
    #     # RUN 버튼 생성 및 위치 설정, 폰트 적용
    #     ttk.Button(self, text="RUN", command=self.toggleRunStatus).place(x=700, y=300, width=150, height=50)
    
    def create_sliders(self):
        # 속도 슬라이더 생성
        ttk.Label(self, text="SPEED").place(x=555, y=480)
        ttk.Scale(self, from_=0, to=1000, orient="horizontal", variable=self.speed_slider_value, command=lambda event=None: self.update_and_send_data(), length=150).place(x=500, y=500)
        speed_entry = ttk.Entry(self, textvariable=self.speed_slider_value, width=3)
        speed_entry.place(x=560, y=530)

        # 가속도 슬라이더 생성
        ttk.Label(self, text="ACCELETION").place(x=740, y=480)
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
            self.run_status.set(0)
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
        self.home_button = ttk.Button(self, text="HOME", command=self.send_home_command)
        self.home_button.place(x=500, y=400, width=350, height=50)

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
    # def received_and_send_to_FOUP(self, FOUP_num, send_data):
    #     if self.serial_port1.readline() == "1_0": #첫 번째 FOUP안에 웨이퍼가 없는 경우
    #         self.message = 1_0
    #     if self.serial_port1.readline() == "1_1": #첫 번째 FOUP안에 웨이퍼가 있는 경우
    #         self.message = 1_1
    #     if self.serial_port1.readline() == "2_0": #두 번째 FOUP안에 웨이퍼가 없는 경우
    #         self.message = 2_0
    #     if self.serial_port1.readline() == "2_1": #두 번째 FOUP안에 웨이퍼가 있는 경우
    #         self.message = 2_1
    #     if FOUP_num == 1 and send_data == 0: #첫 번째 FOUP안에 웨이퍼를 다 뺀 경우
    #         self.serial_port1.write("1_0") # FOUP아두이노에게 1(첫 번째FOUP)_0(웨이퍼 유무)를 보냄
    #     if FOUP_num == 2 and send_data == 1: #두 번째 FOUP안에 웨이퍼를 다 넣은 경우
    #         self.serial_port1.write("2_1") # FOUP아두이노에게 2(두 번째 FOUP)_1(웨이퍼 유무)를 보냄
if __name__ == "__main__":
    app = ScaraRobotGUI()
    app.mainloop()
