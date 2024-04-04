import tkinter as tk
from tkinter import ttk,  IntVar, StringVar
from tkinter import font as tkFont  # 폰트 모듈 임포트
import serial

class ScaraRobotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.geometry("800x600")
        self.title("SCARA Robot Control")
        
        # 시리얼 포트 설정. 적절한 COM 포트로 설정하세요.
        self.serial_port = serial.Serial('COM3', 115200, timeout=1)
        
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

        
        # 폰트를 불러옴
        self.customFont = tkFont.Font(family="Arial", size=10, weight="bold")

        # GUI 컨트롤 생성
        self.create_widgets()
        
        # 저장 포지션 생성
        self.create_save_position_button()

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
        
    def create_widgets(self):
        self.create_slider_with_buttons("J1", -100, 100, 100, 180)
        self.create_slider_with_buttons("J2", -100, 100, 100, 290)
        self.create_slider_with_buttons("J3", -100, 100, 100, 400)
        self.create_slider_with_buttons("Z", -100, 100, 100, 510)

        # 여기에 J2, J3, Z에 대한 비슷한 코드를 추가할 수 있습니다.
        
    def create_slider_with_buttons(self, label, min_val, max_val, x, y):
        # 라벨 위치 설정
        ttk.Label(self, text=f"{label}:", font=self.customFont).place(x=x-50, y=y+5)

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
        # SAVE POSITION 버튼 생성 및 위치 설정
        ttk.Button(self, text="SAVE POSITION", command=self.savePosition).place(x=500, y=300, width=150, height=50)

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
        
if __name__ == "__main__":
    app = ScaraRobotGUI()
    app.mainloop()
