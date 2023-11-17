import RPi.GPIO as GPIO
import threading
import time
import pigpio
import numpy as np
import cv2
import serial
import struct
import global_value
import motor
global_value._init()  #全局变量初始化，初始化字典
key = 0    #陀螺仪函数的全局变量
buff = {}  #陀螺仪函数的全局变量
i_flag = 1 #保存图片的全局变量
i_order_flag = 1
S = serial.Serial("/dev/ttyAMA0", 9600, bytesize=8, stopbits=1, timeout=0.5) #设置机械臂串口
pm = serial.Serial("/dev/ttyAMA3", 9600, timeout=0.1) #设置屏幕串口
MCU = serial.Serial("/dev/ttyAMA1", baudrate=230400) #设置高精度陀螺仪串口
qrCodeDetector = cv2.QRCodeDetector() #设置扫码实例
pi = pigpio.pi()
##################定义引脚##################
PWMA = 27
AIN2 = 17
AIN1 = 18
BIN1 = 16
BIN2 = 20
PWMB = 21
PWMC = 19
CIN1 = 6
CIN2 = 5
DIN1 = 25
DIN2 = 24
PWMD = 23
encoder_pin_A = 22
encoder_pin_B = 26
encoder_pin_C = 13
encoder_pin_D = 12
########################设置参数#######################
start_z = 0  #初始化z轴方向
data = []    #存放扫码数据（123）
ys = []      #存放扫码数据（rgb）
FOOL = 1.05     #摩擦系数
order1 = {} #决赛半成品区顺序
order2 = {}
######################十六进制数组######################
end = bytes.fromhex('ff ff ff')  #串口屏结束符
data0 = b'\xff\x09\x00\x00\x00'  #机械臂动作组0
data1 = b'\xff\x09\x00\x01\x00'  #机械臂动作组1
data2 = b'\xff\x09\x00\x02\x00'  #机械臂动作组2
data3 = b'\xff\x09\x00\x03\x00'  #机械臂动作组3
data4 = b'\xff\x09\x00\x04\x00'  #机械臂动作组4
data5 = b'\xff\x09\x00\x05\x00'  #机械臂动作组5
data6 = b'\xff\x09\x00\x06\x00'  #机械臂动作组6
data7 = b'\xff\x09\x00\x07\x00'  #机械臂动作组7
data8 = b'\xff\x09\x00\x08\x00'  #机械臂动作组8
data9 = b'\xff\x09\x00\x09\x00'  #机械臂动作组9
data10 = b'\xff\x09\x00\x0a\x00' #机械臂动作组10
data11 = b'\xff\x09\x00\x0b\x00' #机械臂动作组11
data12 = b'\xff\x09\x00\x0c\x00' #机械臂动作组12
data13 = b'\xff\x09\x00\x0d\x00' #机械臂动作组13
data14 = b'\xff\x09\x00\x0e\x00' #机械臂动作组14
data15 = b'\xff\x09\x00\x0f\x00' #机械臂动作组15
zero = bytes.fromhex('FF AA 76 00 00') #陀螺仪置零
###########################读取模板图片##########################
template_zp = cv2.imread('pic/template/template_zp.jpg', cv2.IMREAD_GRAYSCALE)
template_cjg = cv2.imread('pic/template/template_cjg.jpg', cv2.IMREAD_GRAYSCALE)
template_zcq = cv2.imread('pic/template/template_zcq.jpg', cv2.IMREAD_GRAYSCALE)
template_wl = cv2.imread('pic/template/template_wl.jpg', cv2.IMREAD_GRAYSCALE)
template_order = cv2.imread('pic/template/template_order.jpg', cv2.IMREAD_GRAYSCALE)     # 用来定位
template_order_2 = cv2.imread('pic/template/template_order_2.jpg', cv2.IMREAD_GRAYSCALE) # 用来识别顺序
template_1_level = cv2.imread('pic/template/template_1_level.jpg', cv2.IMREAD_GRAYSCALE)
template_2_level = cv2.imread('pic/template/template_2_level.jpg', cv2.IMREAD_GRAYSCALE)
template_zp_bullseye = cv2.imread('pic/template/template_zp_bullseye.jpg', cv2.IMREAD_GRAYSCALE)
################################################################
#初始化引脚
def pin_init():
    GPIO.setmode(GPIO.BCM)              #select model
    GPIO_out_list = (PWMA,AIN1,AIN2,BIN1,BIN2,PWMB,PWMC,CIN1,CIN2,DIN1,DIN2,PWMD) #select pin
    # GPIO_in_list = (E1A, JGZ, JGY)
    # GPIO.setup(GPIO_in_list, GPIO.IN)
    GPIO.setup(GPIO_out_list, GPIO.OUT) #set pin's model
    pwm_init(PWMA, 5000, 40000)
    pwm_init(PWMB, 5000, 40000)
    pwm_init(PWMC, 5000, 40000)
    pwm_init(PWMD, 5000, 40000)
################################################################
#初始化pwm引脚
def pwm_init(pin, frequency, totol):
    pi.set_PWM_frequency(pin, frequency)#设定pin号引脚产生的pwm波形的频率为frequency
    pi.set_PWM_range(pin, totol) #指定要把14号引脚上的一个pwm周期分成多少份，这里是分成2000份，这个数据的范围是25-40000
    return totol
################################################################
#设置小车的运动方向
def move(direction):
    if direction == 'front':
        GPIO.output(AIN1,1)
        GPIO.output(AIN2,0)
        GPIO.output(BIN1,1)
        GPIO.output(BIN2,0)
        GPIO.output(CIN1,0)
        GPIO.output(CIN2,1)
        GPIO.output(DIN1,0)
        GPIO.output(DIN2,1)
    elif direction == 'back' :
        GPIO.output(AIN1,0)
        GPIO.output(AIN2,1)
        GPIO.output(BIN1,0)
        GPIO.output(BIN2,1)
        GPIO.output(CIN1,1)
        GPIO.output(CIN2,0)
        GPIO.output(DIN1,1)
        GPIO.output(DIN2,0)
    elif direction == 'xuanzhuanzuo':
        GPIO.output(AIN1,1)
        GPIO.output(AIN2,0)
        GPIO.output(BIN1,0)
        GPIO.output(BIN2,1)
        GPIO.output(CIN1,1)
        GPIO.output(CIN2,0)
        GPIO.output(DIN1,0)
        GPIO.output(DIN2,1)
    elif direction == 'xuanzhuanyou':
        GPIO.output(AIN1,0)
        GPIO.output(AIN2,1)
        GPIO.output(BIN1,1)
        GPIO.output(BIN2,0)
        GPIO.output(CIN1,0)
        GPIO.output(CIN2,1)
        GPIO.output(DIN1,1)
        GPIO.output(DIN2,0)
    elif direction == 'left':
        GPIO.output(AIN1,1)
        GPIO.output(AIN2,0)
        GPIO.output(BIN1,0)
        GPIO.output(BIN2,1)
        GPIO.output(CIN1,0)
        GPIO.output(CIN2,1)
        GPIO.output(DIN1,1)
        GPIO.output(DIN2,0)
    elif direction == 'right':
        GPIO.output(AIN1,0)
        GPIO.output(AIN2,1)
        GPIO.output(BIN1,1)
        GPIO.output(BIN2,0)
        GPIO.output(CIN1,1)
        GPIO.output(CIN2,0)
        GPIO.output(DIN1,0)
        GPIO.output(DIN2,1)
    elif direction == 'stop':
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif direction == 'STOP':
        GPIO.output(AIN1,0)
        GPIO.output(AIN2,0)
        GPIO.output(BIN1,0)
        GPIO.output(BIN2,0)
        GPIO.output(CIN1,0)
        GPIO.output(CIN2,0)
        GPIO.output(DIN1,0)
        GPIO.output(DIN2,0)
    else:
        GPIO.output(AIN1,0)
        GPIO.output(AIN2,0)
        GPIO.output(BIN1,0)
        GPIO.output(BIN2,0)
        GPIO.output(CIN1,0)
        GPIO.output(CIN2,0)
        GPIO.output(DIN1,0)
        GPIO.output(DIN2,0)
################################################################
#扫码
def get_qr_data(input_frame):
    global data
    global ys
    data = []  #123
    ys = []    #rgb
    try:
        data, bbox, straight_qrcode = qrCodeDetector.detectAndDecode(input_frame)
        if data[0] == '3':
            ys.append('b')
        elif data[0] == '2':
            ys.append('g')
        elif data[0] == '1':
            ys.append('r')
        if data[1] == '3':
            ys.append('b')
        elif data[1] == '2':
            ys.append('g')
        elif data[1] == '1':
            ys.append('r')
        if data[2] == '3':
            ys.append('b')
        elif data[2] == '2':
            ys.append('g')
        elif data[2] == '1':
            ys.append('r')
        if data[4] == '3':
            ys.append('b')
        elif data[4] == '2':
            ys.append('g')
        elif data[4] == '1':
            ys.append('r')
        if data[5] == '3':
            ys.append('b')
        elif data[5] == '2':
            ys.append('g')
        elif data[5] == '1':
            ys.append('r')
        if data[6] == '3':
            ys.append('b')
        elif data[6] == '2':
            ys.append('g')
        elif data[6] == '1':
            ys.append('r')
    except:
        return data
################################################################
#返回加速度
def get_acceleration(axis,defValue=None): 
    try :
        return global_value.get_value('jsd')[axis]
    except TypeError:
        return defValue
#返回角度
def get_angle(axis,defValue=None):  
    try :
        return global_value.get_value('JD')[axis]+180
    except TypeError:
        return defValue
################################################################
#旋转
def ToAngle(angle):
    global goForward_flag

    # PID控制参数
    Kp = 1.0
    Ki = 0.1
    Kd = 0.001
    integral = 0
    prev_error = 0

    # 设置速度上限
    max_speed = 50

    # 获取初始时间
    t1 = time.time()

    # 主循环
    while True:
        time.sleep(0.1)
        # 超出5秒退出循环
        t2 = time.time()
        if t2 - t1 > 2.0:
            break

        current_angle = get_angle(2)
        error = angle - current_angle
        integral += error

        # 限制积分
        if integral > 100:
            integral = 100
        elif integral < -100:
            integral = -100

        # 计算导数
        derivative = error - prev_error

        # PID控制输出
        out = Kp * error + Ki * integral + Kd * derivative
        speed_xunazhuan = abs(out)

        # 限制速度
        if speed_xunazhuan > max_speed:
            speed_xunazhuan = max_speed

        # 控制小车转向
        if 0 < angle < 90 and 360 > current_angle > 270:
            if angle - 0.05 < current_angle < angle + 0.05:
                move('stop')
                break
            elif 270 < current_angle < 360:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle < angle - 0.2:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle > angle + 0.2:
                move('xuanzhuanyou')  # 顺时针转圈
        elif 360 > angle > 270 and 0 < current_angle < 90:
            if angle - 0.05 < current_angle < angle + 0.05:
                move('stop')
                break
            elif 0 < current_angle < 90:
                move('xuanzhuanyou')  # 顺时针转圈
            elif current_angle < angle - 0.2:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle > angle + 0.2:
                move('xuanzhuanyou')  # 顺时针转圈
        else:
            if angle - 0.05 < current_angle < angle + 0.05:
                move('stop')
                break
            elif current_angle < angle - 0.2:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle > angle + 0.2:
                move('xuanzhuanyou')  # 顺时针转圈

        # 控制小车速度
        global_value.set_value('targetA', speed_xunazhuan)
        global_value.set_value('targetB', speed_xunazhuan)
        global_value.set_value('targetC', speed_xunazhuan)
        global_value.set_value('targetD', speed_xunazhuan)

        # 更新上次误差
        prev_error = error

        # 打印调试信息
        # print('out: ', out, '  speed: ', speed_xunazhuan)
    
    # 退出循环
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
# 调整小角度
def ToAngle_little(angle):
    global goForward_flag

    # PID控制参数
    Kp = 5.0
    Ki = 0
    Kd = 0
    integral = 0
    prev_error = 0

    # 设置速度上限
    max_speed = 50

    # 获取初始时间
    t1 = time.time()

    # 主循环
    while True:
        time.sleep(0.1)
        # 超出5秒退出循环
        t2 = time.time()
        if t2 - t1 > 1.5:
            break

        current_angle = get_angle(2)
        error = angle - current_angle
        integral += error

        # 限制积分
        if integral > 100:
            integral = 100
        elif integral < -100:
            integral = -100

        # 计算导数
        derivative = error - prev_error

        # PID控制输出
        out = Kp * error + Ki * integral + Kd * derivative
        speed_xunazhuan = abs(out)

        # 限制速度
        if speed_xunazhuan > max_speed:
            speed_xunazhuan = max_speed

        # 控制小车转向
        if 0 < angle < 90 and 360 > current_angle > 270:
            if angle - 0.05 < current_angle < angle + 0.05:
                move('stop')
                break
            elif 270 < current_angle < 360:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle < angle - 0.2:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle > angle + 0.2:
                move('xuanzhuanyou')  # 顺时针转圈
        elif 360 > angle > 270 and 0 < current_angle < 90:
            if angle - 0.05 < current_angle < angle + 0.05:
                move('stop')
                break
            elif 0 < current_angle < 90:
                move('xuanzhuanyou')  # 顺时针转圈
            elif current_angle < angle - 0.2:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle > angle + 0.2:
                move('xuanzhuanyou')  # 顺时针转圈
        else:
            if angle - 0.05 < current_angle < angle + 0.05:
                move('stop')
                break
            elif current_angle < angle - 0.2:
                move('xuanzhuanzuo')  # 逆时针转圈
            elif current_angle > angle + 0.2:
                move('xuanzhuanyou')  # 顺时针转圈

        # 控制小车速度
        global_value.set_value('targetA', speed_xunazhuan)
        global_value.set_value('targetB', speed_xunazhuan)
        global_value.set_value('targetC', speed_xunazhuan)
        global_value.set_value('targetD', speed_xunazhuan)

        # 更新上次误差
        prev_error = error

        # 打印调试信息
        # print('out: ', out, '  speed: ', speed_xunazhuan)
    
    # 退出循环
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
################################################################
#显示数字
def display_num(index, num):
    mes = 'n'+str(index)+'.val='+str(num)   #n0.val=0
    pm.write(bytearray(mes.encode()))
    pm.write(end)
#读取屏幕指令
def Read_pm():
    try:
        mes = pm.read_all()
        a = struct.unpack('<hh', mes)
    except:
        a = (-1, 0)
    return a[0], mes
#刷新屏幕界面
def Page_pm(i):
    mes = 'page page' + str(i)
    pm.write(bytearray(mes.encode()))
    pm.write(end)
################################################################
#爪子摄像头
def getFrame_up():
    cap = cv2.VideoCapture("/dev/first")
    while 1:
        _, frame = cap.read()
        global_value.set_value('frame_up', frame)
################################################################
#行走线程
def MoveTime(dir, t):
    if dir == 'f':
        move('front')
        speed = 80
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                # print('r2')
                global_value.set_value('targetA', speed*1.07)
                global_value.set_value('targetB', speed*0.93)
                global_value.set_value('targetC', speed*0.93)
                global_value.set_value('targetD', speed*1.07)
            elif get_angle(2) - start_z < -1: #向右偏
                # print('r1')
                global_value.set_value('targetA', speed*1.07)
                global_value.set_value('targetB', speed*0.93)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 2: #向左偏
                # print('l2')
                global_value.set_value('targetA', speed*0.93)
                global_value.set_value('targetB', speed*1.07)
                global_value.set_value('targetC', speed*1.07)
                global_value.set_value('targetD', speed*0.93)
            elif get_angle(2) - start_z > 1: #向左偏
                # print('l1')
                global_value.set_value('targetA', speed*0.93)
                global_value.set_value('targetB', speed*1.07)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            else:
                # print('0')
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif dir == 'b':
        move('back')
        speed_l = 82
        speed_r = 78
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed_r)
        global_value.set_value('targetB', speed_l)
        global_value.set_value('targetC', speed_l)
        global_value.set_value('targetD', speed_r)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', speed_r*0.97)
                global_value.set_value('targetB', speed_l*1.03)
                global_value.set_value('targetC', speed_l*1.03)
                global_value.set_value('targetD', speed_r*0.97)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed_r)
                global_value.set_value('targetB', speed_l)
                global_value.set_value('targetC', speed_l*1.03)
                global_value.set_value('targetD', speed_r*0.97)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed_r*1.03)
                global_value.set_value('targetB', speed_l*0.97)
                global_value.set_value('targetC', speed_l*0.97)
                global_value.set_value('targetD', speed_r*1.03)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed_r)
                global_value.set_value('targetB', speed_l)
                global_value.set_value('targetC', speed_l*0.97)
                global_value.set_value('targetD', speed_r*1.03)
            else:
                global_value.set_value('targetA', speed_r)
                global_value.set_value('targetB', speed_l)
                global_value.set_value('targetC', speed_l)
                global_value.set_value('targetD', speed_r)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)     
    elif dir == 'r':
        speed = 80
        move('right')
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -1: #向右偏得更多
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed-2)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed+3)
            elif get_angle(2) - start_z < -0.3: #向右偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed+3)
            elif get_angle(2) - start_z > 1: #向左偏得更多
                global_value.set_value('targetA', speed+3)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed-2)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 0.3: #向左偏
                global_value.set_value('targetA', speed+3)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)        
    elif dir == 'l':
        move('left')
        speed = 50
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -1: #向右偏得更多
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed+speed*0.1)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed-speed*0.1)
            elif get_angle(2) - start_z < -0.3: #向右偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed+speed*0.1)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 1: #向左偏得更多
                global_value.set_value('targetA', speed-speed*0.1)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed+speed*0.1)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 0.3: #向左偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed+speed*0.1)
                global_value.set_value('targetD', speed)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)        
    elif dir == 'xl':
        move('front')
        speed = 15
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', 0)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', speed*1.3)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*0.7)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed*1.3)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed*0.7)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.3)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.7)
                global_value.set_value('targetD', 0)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)        
    elif dir == 'xr':
        move('front')
        speed = 15
        t1 = t2 = time.time()
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*0.7)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.3)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.3)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.3)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*0.7)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.3)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            else:
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)        
    elif dir == 'bxl':
        move('back')
        speed = 15
        t1 = t2 = time.time()
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.3)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*0.7)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.3)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*0.7)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.3)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.3)
            else:
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)        
    elif dir == 'bxr':
        move('back')
        speed = 15
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', 0)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', speed*0.7)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.3)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.3)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed*1.3)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*0.7)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed*1.3)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)  
    elif dir == 'stop':
        move('stop')
        t1 = t2 = time.time()
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)  
    elif dir == 'scan_code':
        move('front')
        global ys
        speed = 50
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if len(ys) == 0:
                frame = cv2.imread('frame_down.jpg')
                get_qr_data(frame)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', speed+3)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed+3)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed+3)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed+3)
                global_value.set_value('targetC', speed+3)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed+3)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif dir == 'cm':
        move('front')
        speed = 40
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', 0)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', speed*1.25)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*0.75)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed*1.25)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed*0.75)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif dir == 'hj_2':
        move('front')
        speed = 40
        t1 = t2 = time.time()
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*0.75)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.25)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.25)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.25)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*0.75)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.25)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            else:
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif dir == 'back_zp':
        move('back')
        speed = 40
        t1 = t2 = time.time()
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', speed)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', speed)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.25)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*0.75)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*1.25)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed*0.75)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.25)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed*1.25)
            else:
                global_value.set_value('targetA', 0)
                global_value.set_value('targetB', speed)
                global_value.set_value('targetC', 0)
                global_value.set_value('targetD', speed)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif dir == 'hj_1':
        move('front')
        speed = 40
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', 0)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', speed*1.25)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*0.75)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed*1.25)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed*0.75)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)    
    elif dir == 'hj_3':
        move('back')
        speed = 40
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', speed)
        global_value.set_value('targetD', 0)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                global_value.set_value('targetA', speed*1.25)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*0.75)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed*1.25)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed*0.75)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            else:
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed)
                global_value.set_value('targetD', 0)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
################################################################
def X_decrease(num):
    motor.count_a = 0
    motor.count_c = 0
    move('front')
    global_value.set_value('targetA', 20)
    global_value.set_value('targetC', 20)
    t2 = t1 = time.time()
    while t2 - t1 < 3:
        t2 = time.time()
        # if global_value.get_value('motorA_') > num:
        #     global_value.set_value('targetA', 0)
        # if global_value.get_value('motorC_') > num:
        #     global_value.set_value('targetC', 0)
        if global_value.get_value('motorA_') > num and global_value.get_value('motorC_') > num:
            break
    motor.count_a = 0
    motor.count_c = 0
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
def X_increase(num):
    motor.count_a = 0
    motor.count_c = 0
    move('back')
    global_value.set_value('targetA', 20)
    global_value.set_value('targetC', 20)
    t2 = t1 = time.time()
    while t2 - t1 < 3:
        t2 = time.time()
        # if global_value.get_value('motorA_') > num:
        #     global_value.set_value('targetA', 0)
        # if global_value.get_value('motorC_') > num:
        #     global_value.set_value('targetC', 0)
        if global_value.get_value('motorA_') > num and global_value.get_value('motorC_') > num:
            break
    motor.count_a = 0
    motor.count_c = 0
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
def Y_decrease(num):
    motor.count_b = 0
    motor.count_d = 0
    move('front')
    global_value.set_value('targetB', 20)
    global_value.set_value('targetD', 20)
    t2 = t1 = time.time()
    while t2 - t1 < 3:
        t2 = time.time()
        # if global_value.get_value('motorB_') > num:
        #     global_value.set_value('targetB', 0)
        # if global_value.get_value('motorD_') > num:
        #     global_value.set_value('targetD', 0)
        if global_value.get_value('motorB_') > num and global_value.get_value('motorD_') > num:
            break
    motor.count_b = 0
    motor.count_d = 0
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
def Y_increase(num):
    motor.count_b = 0
    motor.count_d = 0
    move('back')
    global_value.set_value('targetB', 20)
    global_value.set_value('targetD', 20)
    t2 = t1 = time.time()
    while t2 - t1 < 3:
        t2 = time.time()
        # if global_value.get_value('motorB_') > num:
        #     global_value.set_value('targetB', 0)
        # if global_value.get_value('motorD_') > num:
        #     global_value.set_value('targetD', 0)
        if global_value.get_value('motorB_') > num and global_value.get_value('motorD_') > num:
            break
    motor.count_b = 0
    motor.count_d = 0
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
################################################################
def getPos(): # 初赛看zp
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    h_mask = cv2.inRange(h, 21, 43)
    s_mask = cv2.inRange(s, 23, 104)
    v_mask = cv2.inRange(v, 92, 208)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_zp, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_2(): # cjg 初赛打靶
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    h_mask = cv2.inRange(h, 37, 58)
    s_mask = cv2.inRange(s, 43, 255)
    v_mask = cv2.inRange(v, 46, 255)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_cjg, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_3(): # zcq 初赛码垛
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    h_mask = cv2.inRange(h, 56, 71)
    s_mask = cv2.inRange(s, 50, 255)
    v_mask = cv2.inRange(v, 177, 245)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_zcq, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_4(): # order 决赛看顺序
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    h_mask = cv2.inRange(h, 20, 34)
    s_mask = cv2.inRange(s, 48, 255)
    v_mask = cv2.inRange(v, 48, 255)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_order, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_5(color): # 暂存区取物料第一层
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 3)
        h2_mask = cv2.inRange(h, 175, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
    elif color == 'g':
        h_mask = cv2.inRange(h, 56, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 100, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_1_level, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_6(color): #转盘打靶识别物料位置
    temp = global_value.get_value('frame_up')
    height, width = temp.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=45, scale=1)
    rotated_temp = cv2.warpAffine(src=temp, M=rotate_matrix, dsize=(width, height))
    rotated_temp = rotated_temp[100:300]
    temp_hsv = cv2.cvtColor(rotated_temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 178, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
    elif color == 'g':
        h_mask = cv2.inRange(h, 35, 77)
        s_mask = cv2.inRange(s, 60, 255)
        v_mask = cv2.inRange(v, 60, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 78, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_zp_bullseye, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_7(color): #转盘打靶识别静止或运动
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 178, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
    elif color == 'g':
        h_mask = cv2.inRange(h, 35, 77)
        s_mask = cv2.inRange(s, 60, 255)
        v_mask = cv2.inRange(v, 60, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 78, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_zp_bullseye, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_8(color): # 暂存区取物料第二层
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 3)
        h2_mask = cv2.inRange(h, 175, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
    elif color == 'g':
        h_mask = cv2.inRange(h, 56, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 100, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_2_level, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
################################################################
def get_order(frame):
    order = {}
    _, r_y = ColorRecognition_order('r', frame)
    _, b_y = ColorRecognition_order('b', frame)
    _, g_y = ColorRecognition_order('g', frame)
    if r_y < b_y and r_y < g_y:
        order['r'] = 1
    elif b_y < r_y and b_y < g_y:
        order['b'] = 1
    elif g_y < b_y and g_y < r_y:
        order['g'] = 1
    if r_y < b_y < g_y or g_y < b_y < r_y:
        order['b'] = 2
    elif b_y < r_y < g_y or g_y < r_y < b_y:
        order['r'] = 2
    elif r_y < g_y < b_y or b_y < g_y < r_y:
        order['g'] = 2
    if r_y > b_y and r_y > g_y:
        order['r'] = 3
    elif b_y > r_y and b_y > g_y:
        order['b'] = 3
    elif g_y > b_y and g_y > r_y:
        order['g'] = 3
    return order
################################################################
def adjust_zp_1(X, Y):
    k = 0.3
    i = 1
    x, y = getPos()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-12 < x < X+12 and Y-12 < y < Y+12) is not True:
        t2 = time.time()
        x, y = getPos()
        time.sleep(0.2)
        if y > Y+12:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-12:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-12:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+12:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/zp/zp_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('zp: ', getPos())
################################################################
def adjust_cjg_1(X, Y):
    k = 0.3
    i = 1
    x, y = getPos_2()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-12 < x < X+12 and Y-12 < y < Y+12) is not True:
        t2 = time.time()
        x, y = getPos_2()
        time.sleep(0.2)
        if y > Y+12:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-12:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-12:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+12:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/cjg/cjg_1_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print(getPos_2())
################################################################
def adjust_cjg_2(X, Y):
    k = 0.3
    i = 1
    x, y = getPos_2()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-5 < x < X+5 and Y-5 < y < Y+5) is not True:
        t2 = time.time()
        x, y = getPos_2()
        time.sleep(0.2)
        if y > Y+5:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-5:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-5:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+5:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/cjg/cjg_2_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('cjg: ', getPos_2())
################################################################
def adjust_zcq_1(X, Y):
    k = 0.3
    i = 1
    x, y = getPos_3()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-12 < x < X+12 and Y-12 < y < Y+12) is not True:
        t2 = time.time()
        x, y = getPos_3()
        time.sleep(0.2)
        if y > Y+12:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-12:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-12:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+12:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/zcq/zcq_1_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print(getPos_3())
################################################################
def adjust_zcq_2(X, Y):
    k = 0.30
    i = 1
    x, y = getPos_3()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-7 < x < X+7 and Y-7 < y < Y+7) is not True:
        t2 = time.time()
        x, y = getPos_3()
        time.sleep(0.2)
        if y > Y+7:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-7:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-7:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+7:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/zcq/zcq_2_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('zcq: ', getPos_3())
################################################################
def adjust_order(X, Y):
    k = 0.30
    i = 1
    x, y = getPos_4()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-7 < x < X+7 and Y-7 < y < Y+7) is not True:
        t2 = time.time()
        x, y = getPos_4()
        time.sleep(0.2)
        if y > Y+7:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-7:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-7:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+7:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/order/order_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('order: ', getPos_4())
################################################################
def adjust_jjg_1(X, Y, ys):
    k = 0.30
    i = 1
    x, y = getPos_5(ys)
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-7 < x < X+7 and Y-7 < y < Y+7) is not True:
        t2 = time.time()
        x, y = getPos_5(ys)
        time.sleep(0.2)
        if y > Y+7:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-7:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-7:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+7:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/jjg1/jjg_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('jjg: ', getPos_5(ys))
################################################################
def adjust_jjg_2(X, Y, ys):
    k = 0.30
    i = 1
    x, y = getPos_5(ys)
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while (X-7 < x < X+7 and Y-7 < y < Y+7) is not True:
        t2 = time.time()
        x, y = getPos_5(ys)
        time.sleep(0.2)
        if y > Y+7:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-7:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-7:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+7:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/jjg2/jjg_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('jjg: ', getPos_5(ys))
################################################################
# 初使动作
def arm_initialize():
    # S.write(bytes.fromhex('ff 01 0b 14 00'))
    # S.write(bytes.fromhex('ff 02 0b c6 04'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 0a 35 05'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 02 09 82 06'))
    S.write(bytes.fromhex('ff 01 08 14 00'))
    S.write(bytes.fromhex('ff 02 08 6c 05'))
# 看转盘动作
def arm_aim_turntable():
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b 20 03'))
    S.write(bytes.fromhex('ff 01 0a 0a 00'))
    S.write(bytes.fromhex('ff 02 0a 57 04'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 98 07'))
    S.write(bytes.fromhex('ff 01 08 14 00'))
    S.write(bytes.fromhex('ff 02 08 2b 03'))
# 看物料动作
def arm_see_wl():
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b 20 03'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a c6 04'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 35 05'))
    S.write(bytes.fromhex('ff 01 08 28 00'))
    S.write(bytes.fromhex('ff 02 08 2b 03'))
# 抓取过渡1
def arm_grab_interim():
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 57 04'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 dc 05'))
    S.write(bytes.fromhex('ff 01 08 28 00'))
    S.write(bytes.fromhex('ff 02 08 c7 02'))
# 看绿靶动作
def arm_aim_bullseye():
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b 78 03'))
    S.write(bytes.fromhex('ff 01 0a 0a 00'))
    S.write(bytes.fromhex('ff 02 0a 13 06'))
    S.write(bytes.fromhex('ff 01 09 10 00'))
    S.write(bytes.fromhex('ff 02 09 60 07'))
    S.write(bytes.fromhex('ff 01 08 10 00'))
    S.write(bytes.fromhex('ff 02 08 2b 03'))
# 抓取动作
def arm_grab():
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b f2 05'))
# 放手动作
def arm_losse():
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b 8e 04'))
def arm_interim():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 af 06'))
    S.write(bytes.fromhex('ff 01 09 0c 00'))
    S.write(bytes.fromhex('ff 02 09 6c 05'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a fd 04'))
    S.write(bytes.fromhex('ff 02 08 fe 03'))
    S.write(bytes.fromhex('ff 02 09 f2 04'))
    S.write(bytes.fromhex('ff 02 0a 82 07'))
# 抓转盘
def grab_wl_up():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 20 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 f1 06'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
# 抓转盘低版
def grab_wl_up_lower():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 20 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 a4 06'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 4b 06'))
# 抓绿色
def grab_wl_middle():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 fe 02'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 40 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
# 抓绿色低版
def grab_wl_middle_lower():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 c7 02'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 8e 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
# 抓蓝色
def grab_wl_down():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 84 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 57 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
# 抓蓝色低版
def grab_wl_down_lower():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 9a 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 8e 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
# 决赛看顺序
def arm_see_order():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 1f 04'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 6c 05'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 8e 04'))
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b 20 03'))
# 一层抓取
def first_level_1():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 63 02'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 e7 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 8d 07'))
def first_level_2():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 41 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 61 05'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a d0 07'))
def first_level_3():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 14 04'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 f2 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 82 07'))
# 二层抓取
def second_level_1():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 9a 02'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 1f 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 34 06'))
def second_level_2():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 4c 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 8e 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 6c 06'))
def second_level_3():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 d1 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 1f 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 34 06'))
# 机械臂暂停
def arm_stop():
    S.write(bytes.fromhex('ff 0b 00 01 00'))
# 机械臂恢复
def arm_recover():
    S.write(bytes.fromhex('ff 0b 00 00 00'))
# 抓着看靶心
def grab_see():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 2b 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 29 07'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 57 04'))
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b 08 06'))
################################################################
#颜色识别
def ColorRecognition(color, img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == 'b':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 100, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
        cv2.imwrite('pic/color_sample/sample_blue'+str(i_flag)+'.jpg', img)
    elif color == 'r':
        h, s, v = cv2.split(img_hsv)
        h1_mask = cv2.inRange(h, 0, 3)
        h2_mask = cv2.inRange(h, 175, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
        result = cv2.matchTemplate(mask, template_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
        cv2.imwrite('pic/color_sample/sample_red'+str(i_flag)+'.jpg', img)
    elif color == 'g':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 56, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
        cv2.imwrite('pic/color_sample/sample_green'+str(i_flag)+'.jpg', img)
    else:
        (startX, startY) = (0, 0)
    return (startX, startY)
def ColorRecognition_order(color, img):
    global i_order_flag
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == 'b':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 100, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_order_2, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
    elif color == 'r':
        h, s, v = cv2.split(img_hsv)
        h1_mask = cv2.inRange(h, 0, 3)
        h2_mask = cv2.inRange(h, 175, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask1 = h1_mask & s_mask & v_mask
        mask2 = h2_mask & s_mask & v_mask
        mask = mask1 | mask2
        result = cv2.matchTemplate(mask, template_order_2, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
    elif color == 'g':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 56, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_order_2, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
    else:
        (startX, startY) = (0, 0)
    cv2.imwrite('pic/color_sample/sample_zp_order_'+str(i_order_flag)+'.jpg', img)
    i_order_flag = i_order_flag +1
    return (startX, startY)
################################################################
# 初赛第一次抓转盘
def grab_zp():
    #######第1个物料#######
    if MoveOrStatic(ys[0]) == 1:
        print('special condition!')
        time.sleep(4)
    while MoveOrStatic(ys[0]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    order_temp = get_order(frame)
    if order_temp[ys[0]] == 1:
        S.write(data3)
    elif order_temp[ys[0]] == 2:
        S.write(data2)
    elif order_temp[ys[0]] == 3:
        S.write(data1)
    time.sleep(0.5)
    if ys[0] == 'r':
        S.write(data13)
        time.sleep(3.3)
    elif ys[0] == 'g':
        S.write(data14)
        time.sleep(3.6)
    elif ys[0] == 'b':
        S.write(data15)
        time.sleep(4.2)
    #######第2个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[1]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    order_temp = get_order(frame)
    if order_temp[ys[1]] == 1:
        S.write(data3)
    elif order_temp[ys[1]] == 2:
        S.write(data2)
    elif order_temp[ys[1]] == 3:
        S.write(data1)
    time.sleep(0.5)
    if ys[1] == 'r':
        S.write(data13)
        time.sleep(3.3)
    elif ys[1] == 'g':
        S.write(data14)
        time.sleep(3.6)
    elif ys[1] == 'b':
        S.write(data15)
        time.sleep(4.2)
    #######第3个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[2]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    order_temp = get_order(frame)
    if order_temp[ys[2]] == 1:
        S.write(data3)
    elif order_temp[ys[2]] == 2:
        S.write(data2)
    elif order_temp[ys[2]] == 3:
        S.write(data1)
    time.sleep(0.5)
    if ys[2] == 'r':
        S.write(data13)
        time.sleep(2.3)
    elif ys[2] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[2] == 'b':
        S.write(data15)
        time.sleep(3.2)
    arm_initialize()
# 初赛第二次抓转盘
def grab_zp_2():
    #######第1个物料#######
    if MoveOrStatic(ys[3]) == 1:
        print('special condition!')
        time.sleep(4)
    while MoveOrStatic(ys[3]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    order_temp = get_order(frame)
    if order_temp[ys[3]] == 1:
        S.write(data3)
    elif order_temp[ys[3]] == 2:
        S.write(data2)
    elif order_temp[ys[3]] == 3:
        S.write(data1)
    time.sleep(0.5)
    if ys[3] == 'r':
        S.write(data13)
        time.sleep(3.3)
    elif ys[3] == 'g':
        S.write(data14)
        time.sleep(3.6)
    elif ys[3] == 'b':
        S.write(data15)
        time.sleep(4.2)
    #######第2个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[4]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    order_temp = get_order(frame)
    if order_temp[ys[4]] == 1:
        S.write(data3)
    elif order_temp[ys[4]] == 2:
        S.write(data2)
    elif order_temp[ys[4]] == 3:
        S.write(data1)
    time.sleep(0.5)
    if ys[4] == 'r':
        S.write(data13)
        time.sleep(3.3)
    elif ys[4] == 'g':
        S.write(data14)
        time.sleep(3.6)
    elif ys[4] == 'b':
        S.write(data15)
        time.sleep(4.2)
    #######第3个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[5]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    order_temp = get_order(frame)
    if order_temp[ys[5]] == 1:
        S.write(data3)
    elif order_temp[ys[5]] == 2:
        S.write(data2)
    elif order_temp[ys[5]] == 3:
        S.write(data1)
    time.sleep(0.5)
    if ys[5] == 'r':
        S.write(data13)
        time.sleep(2.3)
    elif ys[5] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[5] == 'b':
        S.write(data15)
        time.sleep(3.2)
    arm_initialize()
################################################################
# 初赛第一次粗加工打靶并抓取
def arm_cjg():
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[0] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[0] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[1] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[1] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[2] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[2] == 'b':
        S.write(data10)
        time.sleep(4.4)
        
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data7)
        time.sleep(3.4)
    elif ys[0] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[0] == 'b':
        S.write(data9)
        time.sleep(3.9)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data7)
        time.sleep(3.4)
    elif ys[1] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[1] == 'b':
        S.write(data9)
        time.sleep(3.9)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[2] == 'g':
        S.write(data8)
        time.sleep(3.0)
    elif ys[2] == 'b':
        S.write(data9)
        time.sleep(2.9)
    arm_initialize()  
# 初赛第二次粗加工打靶并抓取
def arm_cjg_2():
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data12)
        time.sleep(4.0)
    elif ys[3] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[3] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data12)
        time.sleep(4.0)
    elif ys[4] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[4] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data12)
        time.sleep(4.0)
    elif ys[5] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[5] == 'b':
        S.write(data10)
        time.sleep(4.4)
        
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data7)
        time.sleep(3.4)
    elif ys[3] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[3] == 'b':
        S.write(data9)
        time.sleep(3.9)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data7)
        time.sleep(3.4)
    elif ys[4] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[4] == 'b':
        S.write(data9)
        time.sleep(3.9)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[5] == 'g':
        S.write(data8)
        time.sleep(3.0)
    elif ys[5] == 'b':
        S.write(data9)
        time.sleep(2.9)
    arm_initialize()
################################################################
# 初赛第一次暂存区打靶
def arm_zcq():
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data12)
        time.sleep(4.0)
    elif ys[0] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[0] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data12)
        time.sleep(4.0)
    elif ys[1] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[1] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data12)
        time.sleep(4.0)
    elif ys[2] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[2] == 'b':
        S.write(data10)
        time.sleep(4.4)
    arm_initialize()
# 初赛第二次暂存区打靶
def arm_zcq_2():
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data6)
        time.sleep(3.6)
    elif ys[3] == 'g':
        S.write(data5)
        time.sleep(3.9)
    elif ys[3] == 'b':
        S.write(data4)
        time.sleep(4.1)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data6)
        time.sleep(3.6)
    elif ys[4] == 'g':
        S.write(data5)
        time.sleep(3.9)
    elif ys[4] == 'b':
        S.write(data4)
        time.sleep(4.1)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data6)
        time.sleep(3.6)
    elif ys[5] == 'g':
        S.write(data5)
        time.sleep(3.9)
    elif ys[5] == 'b':
        S.write(data4)
        time.sleep(4.1)
    arm_initialize()  
################################################################
# 决赛抓取第一层物料
def grab_zcq_first_level():
    if order1[ys[0]] == 1:
        first_level_1()
    elif order1[ys[0]] == 2:
        first_level_2()
    elif order1[ys[0]] == 3:
        first_level_3()
    time.sleep(0.5)
    if ys[0] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[0] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[0] == 'b':
        S.write(data9)
        time.sleep(4.05)
        
    if order1[ys[1]] == 1:
        first_level_1()
    elif order1[ys[1]] == 2:
        first_level_2()
    elif order1[ys[1]] == 3:
        first_level_3()
    time.sleep(0.5)
    if ys[1] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[1] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[1] == 'b':
        S.write(data9)
        time.sleep(4.05)

    if order1[ys[2]] == 1:
        first_level_1()
    elif order1[ys[2]] == 2:
        first_level_2()
    elif order1[ys[2]] == 3:
        first_level_3()
    time.sleep(0.5)
    if ys[2] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[2] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[2] == 'b':
        S.write(data9)
        time.sleep(4.05)
# 决赛抓取第二层物料
def grab_zcq_second_level():
    if order2[ys[3]] == 1:
        second_level_1()
    elif order2[ys[3]] == 2:
        second_level_2()
    elif order2[ys[3]] == 3:
        second_level_3()
    time.sleep(0.5)
    if ys[3] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[3] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[3] == 'b':
        S.write(data9)
        time.sleep(4.05)
        
    if order2[ys[4]] == 1:
        second_level_1()
    elif order2[ys[4]] == 2:
        second_level_2()
    elif order2[ys[4]] == 3:
        second_level_3()
    time.sleep(0.5)
    if ys[4] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[4] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[4] == 'b':
        S.write(data9)
        time.sleep(4.05)

    if order2[ys[5]] == 1:
        second_level_1()
    elif order2[ys[5]] == 2:
        second_level_2()
    elif order2[ys[5]] == 3:
        second_level_3()
    time.sleep(0.5)
    if ys[5] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[5] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[5] == 'b':
        S.write(data9)
        time.sleep(4.05)
################################################################
# 决赛第一次打靶精加工区域
def arm_jjg_1():
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[0] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[0] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[1] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[1] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[2] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[2] == 'b':
        S.write(data10)
        time.sleep(4.4)
        
    arm_see_wl()
    time.sleep(0.5)
    
    #######第1个物料#######
    if ys[0] == 'r':
        first_level_1()
    elif ys[0] == 'g':
        first_level_2()
    elif ys[0] == 'b':
        first_level_3()
    time.sleep(0.5)
    if ys[0] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[0] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[0] == 'b':
        S.write(data9)
        time.sleep(4.05)
    #######第2个物料#######
    if ys[1] == 'r':
        first_level_1()
    elif ys[1] == 'g':
        first_level_2()
    elif ys[1] == 'b':
        first_level_3()
    time.sleep(0.5)
    if ys[1] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[1] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[1] == 'b':
        S.write(data9)
        time.sleep(4.05)
    #######第3个物料#######
    if ys[2] == 'r':
        first_level_1()
    elif ys[2] == 'g':
        first_level_2()
    elif ys[2] == 'b':
        first_level_3()
    time.sleep(0.5)
    if ys[2] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[2] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[2] == 'b':
        S.write(data9)
        time.sleep(4.05)
    arm_initialize()  
# 决赛第二次打靶精加工区域
def arm_jjg_2():
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[3] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[3] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[4] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[4] == 'b':
        S.write(data10)
        time.sleep(4.4)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data12)
        time.sleep(3.8)
    elif ys[5] == 'g':
        S.write(data11)
        time.sleep(4.3)
    elif ys[5] == 'b':
        S.write(data10)
        time.sleep(4.4)
        
    arm_see_wl()
    time.sleep(0.5)
    
    #######第1个物料#######
    if ys[3] == 'r':
        first_level_1()
    elif ys[3] == 'g':
        first_level_2()
    elif ys[3] == 'b':
        first_level_3()
    time.sleep(0.5)
    if ys[3] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[3] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[3] == 'b':
        S.write(data9)
        time.sleep(4.05)
    #######第2个物料#######
    if ys[4] == 'r':
        first_level_1()
    elif ys[4] == 'g':
        first_level_2()
    elif ys[4] == 'b':
        first_level_3()
    time.sleep(0.5)
    if ys[4] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[4] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[4] == 'b':
        S.write(data9)
        time.sleep(4.05)
    #######第3个物料#######
    if ys[5] == 'r':
        first_level_1()
    elif ys[5] == 'g':
        first_level_2()
    elif ys[5] == 'b':
        first_level_3()
    time.sleep(0.5)
    if ys[5] == 'r':
        S.write(data7)
        time.sleep(2.9)
    elif ys[5] == 'g':
        S.write(data8)
        time.sleep(3.5)
    elif ys[5] == 'b':
        S.write(data9)
        time.sleep(4.05)
    arm_initialize()  
################################################################
#判断转盘是运动还是静止
def MoveOrStatic(color):
    frame1 = global_value.get_value('frame_up')
    x1, y1 = ColorRecognition(color, frame1)
    frame2 = global_value.get_value('frame_up')
    x2, y2 = ColorRecognition(color, frame2)
    # return abs(x2-x1) + abs(y2-y1)
    if abs(x2-x1) + abs(y2-y1) >= 2:
        return 0   #move
    elif abs(x2-x1) + abs(y2-y1) < 2:
        return 1   #static
def MoveOrStatic_2():
    frame1 = global_value.get_value('frame')
    x1, y1 = getPos_7('r')
    frame2 = global_value.get_value('frame')
    x2, y2 = getPos_7('r')
    if abs(x2-x1) + abs(y2-y1) >= 3:
        return 'move'   #move
    elif abs(x2-x1) + abs(y2-y1) < 3:
        return 'static'   #static
################################################################
def arm_cpq_1():
    if ys[0] == 'b':
        S.write(data15)
        time.sleep(3)
    elif ys[0] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[0] == 'r':
        S.write(data13)
        time.sleep(2.0)
    arm_stop()
    x, y = getPos_6(ys[0])
    while 1:
        x, y = getPos_6(ys[0])
        if MoveOrStatic_2() == 'static':
            x, y = getPos_6(ys[0])
            if 261 > x > 180 :
                break
    cv2.imwrite('t1.jpg', global_value.get_value('frame_up'))
    arm_recover()
    time.sleep(1.2)

    if ys[1] == 'b':
        S.write(data15)
        time.sleep(3)
    elif ys[1] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[1] == 'r':
        S.write(data13)
        time.sleep(2.0)
    arm_stop()
    x, y = getPos_6(ys[1])
    while 1:
        x, y = getPos_6(ys[1])
        if MoveOrStatic_2() == 'static':
            x, y = getPos_6(ys[1])
            if 261 > x > 180 :
                break
    cv2.imwrite('t2.jpg', global_value.get_value('frame_up'))
    arm_recover()
    time.sleep(1.2)

    if ys[2] == 'b':
        S.write(data15)
        time.sleep(3)
    elif ys[2] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[2] == 'r':
        S.write(data13)
        time.sleep(2.0)
    arm_stop()
    x, y = getPos_6(ys[2])
    while 1:
        x, y = getPos_6(ys[2])
        if MoveOrStatic_2() == 'static':
            x, y = getPos_6(ys[2])
            if 261 > x > 180 :
                break
    cv2.imwrite('t3.jpg', global_value.get_value('frame_up'))
    arm_recover()
    time.sleep(1.5)
    arm_initialize()
def arm_cpq_2():
    if ys[3] == 'b':
        S.write(data15)
        time.sleep(3)
    elif ys[3] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[3] == 'r':
        S.write(data13)
        time.sleep(2.0)
    arm_stop()
    x, y = getPos_6(ys[3])
    while 1:
        x, y = getPos_6(ys[3])
        if MoveOrStatic_2() == 'static':
            x, y = getPos_6(ys[3])
            if 261 > x > 180 :
                break
    cv2.imwrite('t1.jpg', global_value.get_value('frame_up'))
    arm_recover()
    time.sleep(1.2)

    if ys[4] == 'b':
        S.write(data15)
        time.sleep(3)
    elif ys[4] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[4] == 'r':
        S.write(data13)
        time.sleep(2.0)
    arm_stop()
    x, y = getPos_6(ys[4])
    while 1:
        x, y = getPos_6(ys[4])
        if MoveOrStatic_2() == 'static':
            x, y = getPos_6(ys[4])
            if 261 > x > 180 :
                break
    cv2.imwrite('t2.jpg', global_value.get_value('frame_up'))
    arm_recover()
    time.sleep(1.2)

    if ys[5] == 'b':
        S.write(data15)
        time.sleep(3)
    elif ys[5] == 'g':
        S.write(data14)
        time.sleep(2.6)
    elif ys[5] == 'r':
        S.write(data13)
        time.sleep(2.0)
    arm_stop()
    x, y = getPos_6(ys[5])
    while 1:
        x, y = getPos_6(ys[5])
        if MoveOrStatic_2() == 'static':
            x, y = getPos_6(ys[5])
            if 261 > x > 180 :
                break
    cv2.imwrite('t3.jpg', global_value.get_value('frame_up'))
    arm_recover()
    time.sleep(1.5)
    arm_initialize()
################################################################
#定义进程
IMG_up = threading.Thread(target=getFrame_up)
MOTOR = threading.Thread(target=motor.GetSpeed)
CONTROL_A= threading.Thread(target=motor.SpeedControl_A)
CONTROL_B= threading.Thread(target=motor.SpeedControl_B)
CONTROL_C= threading.Thread(target=motor.SpeedControl_C)
CONTROL_D= threading.Thread(target=motor.SpeedControl_D)
################################################################
# 开启进程
MOTOR.start()
CONTROL_A.start()
CONTROL_B.start()
CONTROL_C.start()
CONTROL_D.start()
################################################################
zp = cv2.imread('pic/pic_sample/zp.jpg')
temp_hsv = cv2.cvtColor(zp, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 21, 43)
s_mask = cv2.inRange(s, 23, 104)
v_mask = cv2.inRange(v, 92, 208)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_zp, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
zp_x, zp_y = maxLoc
print('zp: ', maxLoc)
################################################################
cjg = cv2.imread('pic/pic_sample/cjg.jpg')
temp_hsv = cv2.cvtColor(cjg, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 37, 58)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_cjg, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
cjg_x, cjg_y = maxLoc
print('cjg: ', maxLoc)
################################################################
zcq1 = cv2.imread('pic/pic_sample/zcq1.jpg')
temp_hsv = cv2.cvtColor(zcq1, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 37, 58)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_cjg, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
zcq1_x, zcq1_y = maxLoc
print('zcq1: ', maxLoc)
################################################################
zcq2 = cv2.imread('pic/pic_sample/zcq2.jpg')
temp_hsv = cv2.cvtColor(zcq2, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 56, 77)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_zcq, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
zcq2_x, zcq2_y = maxLoc
print('zcq2: ', maxLoc)
################################################################
order = cv2.imread('pic/pic_sample/order.jpg')
temp_hsv = cv2.cvtColor(order, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 20, 34)
s_mask = cv2.inRange(s, 48, 255)
v_mask = cv2.inRange(v, 48, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_order, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
order_x, order_y = maxLoc
print('order: ', maxLoc)
################################################################
first_level = cv2.imread('pic/pic_sample/1_level.jpg')
temp_hsv = cv2.cvtColor(first_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 45, 80)
s_mask = cv2.inRange(s, 60, 255)
v_mask = cv2.inRange(v, 60, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_1_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
first_level_x, first_level_y = maxLoc
print('first_level: ', maxLoc)
################################################################
second_level = cv2.imread('pic/pic_sample/2_level.jpg')
temp_hsv = cv2.cvtColor(second_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 45, 80)
s_mask = cv2.inRange(s, 60, 255)
v_mask = cv2.inRange(v, 60, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_2_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
second_level_x, second_level_y = maxLoc
print('second_level: ', maxLoc)
################################################################
pin_init()
print('ready')
Page_pm(0)
_ = pm.read_all()
while True:
    a, mes = Read_pm()
    time.sleep(0.01)
    if a == 1: # 初赛
        print('chu sai')
        arm_initialize()
        while True:
            a, mes = Read_pm()
            time.sleep(0.01)
            if a == 2:
                print('MCU write zero')
                MCU.write(zero)
            elif a == 1:
                print('preperation')
                # 开启底部摄像头
                cap_temp = cv2.VideoCapture("/dev/second")
                # 刷新屏幕
                for i in range(6):
                    display_num(i, 0)#屏幕清零
                    time.sleep(0.01)
                # 获取角度
                start_z = None   
                while start_z == None:  #初始化角度Z，否则会返回None
                    start_z = get_angle(2)
                first_z = start_z
                second_z = start_z + 90 #获取角度
                if second_z > 360:
                    second_z = second_z - 360
                third_z = start_z + 180 #获取角度
                if third_z > 360:
                    third_z =  third_z - 360
                fouth_z = start_z - 90  #获取角度
                if fouth_z < 0:
                    fouth_z =  360 + fouth_z
                while True:
                    a, mes = Read_pm()
                    time.sleep(0.01)
                    if a == 1:
                        # 正式开始
                        global_value.set_value('model', 1)
                        MoveTime('cm', 0.8)
                        global_value.set_value('model', 0)
                        arm_initialize()
                        MoveTime('f', 0.73)
                        while len(ys) == 0:
                            if cap_temp.isOpened():
                                ret, frame = cap_temp.read()
                                if ret:
                                    get_qr_data(frame)
                        if len(data) != 0:
                            for i in range(3):
                                display_num(i, int(data[i]))
                            for i in range(3,6):
                                display_num(i, int(data[i+1]))
                        cap_temp.release() # 关闭底部摄像头
                        ###################################
                        # 定位转盘
                        IMG_up.start()
                        global_value.set_value('frame_up_flag', 1)
                        MoveTime('f', 1.4)
                        time.sleep(0.5)
                        arm_aim_turntable()
                        print(ys)
                        adjust_zp_1(zp_x, zp_y) # 定位
                        if first_z-3 < get_angle(2) < first_z+3:
                            ToAngle_little(first_z)
                        else:
                            ToAngle(first_z)
                            ToAngle_little(first_z)
                        adjust_zp_1(zp_x, zp_y) # 定位
                        ###################################
                        # 抓物料
                        arm_see_wl()
                        time.sleep(1)
                        grab_zp()
                        ###################################
                        # 抓完物料定位粗加工区
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.43)
                        MoveTime('cm', 0.7)
                        ToAngle(second_z)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        start_z = second_z
                        MoveTime('f', 1.30)
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        adjust_cjg_2(cjg_x, cjg_y)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        adjust_cjg_2(cjg_x, cjg_y)
                        ###################################
                        # 打靶粗加工
                        arm_cjg()
                        ###################################
                        # 打完靶后定位暂存区
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.9)
                        MoveTime('cm', 1.0)
                        ToAngle(third_z)
                        if third_z-2 < get_angle(2) < third_z+2:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        start_z = third_z
                        MoveTime('f', 1.1)
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        adjust_cjg_2(zcq1_x, zcq1_y)
                        if third_z-3 < get_angle(2) < third_z+3:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        adjust_cjg_2(zcq1_x, zcq1_y)
                        ###################################
                        # 打靶暂存区
                        arm_zcq()
                        ###################################
                        # 放置好物料后返回转盘
                        i_flag = 2
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.2)
                        MoveTime('back_zp', 0.8)
                        ToAngle(second_z)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        start_z = second_z
                        MoveTime('b', 2.30)
                        MoveTime('back_zp', 0.8)
                        ToAngle(first_z)
                        if first_z-3 < get_angle(2) < first_z+3:
                            ToAngle_little(first_z)
                        else:
                            ToAngle(first_z)
                            ToAngle_little(first_z)
                        start_z = first_z
                        MoveTime('b', 0.4)
                        time.sleep(0.5)
                        ###################################
                        # 定位转盘
                        global_value.set_value('model',1)
                        arm_aim_turntable()
                        adjust_zp_1(zp_x, zp_y) #定位
                        if first_z-3 < get_angle(2) < first_z+3:
                            ToAngle_little(first_z)
                        else:
                            ToAngle(first_z)
                            ToAngle_little(first_z)
                        adjust_zp_1(zp_x, zp_y) #定位
                        ###################################
                        # 抓取第二波物料
                        arm_see_wl()
                        time.sleep(1)
                        grab_zp_2()
                        ###################################
                        # 定位粗加工区
                        arm_initialize()
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.43)
                        MoveTime('cm', 0.8)
                        start_z = second_z
                        ToAngle(second_z)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        MoveTime('f', 1.30)
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        adjust_cjg_2(cjg_x, cjg_y)
                        if second_z-3 < get_angle(2) < second_z+3:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        adjust_cjg_2(cjg_x, cjg_y)
                        ###################################
                        # 打靶
                        arm_cjg_2()
                        ###################################
                        # 定位暂存区
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.9)
                        MoveTime('cm', 1.0)
                        start_z = third_z
                        ToAngle(third_z)
                        if third_z-2 < get_angle(2) < third_z+2:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        MoveTime('f', 1.1)
                        time.sleep(0.5)
                        global_value.set_value('model', 1)
                        arm_aim_bullseye()
                        ToAngle_little(third_z)
                        adjust_zcq_2(zcq2_x, zcq2_y)
                        ###################################
                        # 打靶
                        arm_zcq_2()
                        ###################################
                        global_value.set_value('model', 0)
                        MoveTime('f', 1.40)
                        MoveTime('hj_1', 0.8)
                        ToAngle(fouth_z)
                        if fouth_z-2 < get_angle(2) < fouth_z+2:
                            ToAngle_little(fouth_z)
                        else:
                            ToAngle(fouth_z)
                            ToAngle_little(fouth_z)
                        start_z = fouth_z
                        MoveTime('f', 2.6)
                        MoveTime('hj_2', 1.0)
            elif a == 0:
                print('exit chusai')
                break
    elif a == 2: # 决赛
        print('jue sai')
        arm_initialize()
        while True:
            a, mes = Read_pm()
            time.sleep(0.01)
            if a == 2:
                print('MCU write zero')
                MCU.write(zero)
            elif a == 1:
                print('preperation')
                # 开启底部摄像头
                cap_temp = cv2.VideoCapture("/dev/second")
                # 刷新屏幕
                for i in range(6):
                    display_num(i, 0)#屏幕清零
                    time.sleep(0.01)
                # 获取角度
                start_z = None   
                while start_z == None:  #初始化角度Z，否则会返回None
                    start_z = get_angle(2)
                first_z = start_z
                second_z = start_z + 90 #获取角度
                if second_z > 360:
                    second_z = second_z - 360
                third_z = start_z + 180 #获取角度
                if third_z > 360:
                    third_z =  third_z - 360
                fouth_z = start_z - 90  #获取角度
                if fouth_z < 0:
                    fouth_z =  360 + fouth_z
                while True:
                    a, mes = Read_pm()
                    time.sleep(0.01)
                    if a == 1:
                        # 正式开始
                        global_value.set_value('model', 1)
                        MoveTime('cm', 0.8)
                        global_value.set_value('model', 0)
                        arm_initialize()
                        MoveTime('f', 0.73)
                        while len(ys) == 0:
                            if cap_temp.isOpened():
                                ret, frame = cap_temp.read()
                                if ret:
                                    get_qr_data(frame)
                        if len(data) != 0:
                            for i in range(3):
                                display_num(i, int(data[i]))
                            for i in range(3,6):
                                display_num(i, int(data[i+1]))
                        cap_temp.release() # 关闭底部摄像头
                        ###################################
                        # 去暂存区
                        IMG_up.start()
                        global_value.set_value('frame_up_flag', 1)
                        MoveTime('b', 0.80)
                        MoveTime('back_zp', 0.8)
                        ToAngle(second_z)
                        if second_z-3 < get_angle(2) < second_z+3:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        start_z = second_z
                        MoveTime('f', 2.20)
                        MoveTime('hj_2', 0.8)
                        ToAngle(third_z)
                        if third_z-2 < get_angle(2) < third_z+2:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        start_z = third_z
                        MoveTime('b', 1.60)
                        arm_see_order()
                        ###################################
                        # 对准双层物料看顺序
                        adjust_order(order_x, order_y)
                        if third_z-2 < get_angle(2) < third_z+2:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        adjust_order(order_x, order_y)
                        time.sleep(0.5)
                        ###################################
                        # 看物料顺序
                        Order_temp = global_value.get_value('frame_up')
                        order1_pic = Order_temp[:, 230:400]
                        order2_pic = Order_temp[:, 0:230]

                        cv2.imwrite('pic/order1.jpg', order1_pic)
                        cv2.imwrite('pic/order2.jpg', order2_pic)
                        
                        order1 = get_order(order1_pic)
                        order2 = get_order(order2_pic)
                        
                        print(order1)
                        print(order2)
                        ###################################
                        # 对准暂存区
                        arm_aim_bullseye()
                        adjust_jjg_1(first_level_x, first_level_y, list(order1.keys())[1])
                        if third_z-2 < get_angle(2) < third_z+2:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        adjust_jjg_1(first_level_x, first_level_y, list(order1.keys())[1])
                        ###################################
                        # 抓暂存区物料
                        grab_zcq_first_level()
                        ###################################
                        # 抓完物料去精加工区
                        global_value.set_value('model', 0)
                        MoveTime('b', 0.95)
                        MoveTime('back_zp', 1.0)
                        ToAngle(second_z)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        start_z = second_z
                        MoveTime('b', 0.9)
                        ###################################
                        # 对准精加工
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        adjust_cjg_2(cjg_x, cjg_y)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        adjust_cjg_2(cjg_x, cjg_y)
                        ###################################
                        # 打靶精加工
                        arm_jjg_1()
                        ###################################
                        # 打完靶后去转盘
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.3)
                        MoveTime('back_zp', 0.8)
                        ToAngle(first_z)
                        if first_z-2 < get_angle(2) < first_z+2:
                            ToAngle_little(first_z)
                        else:
                            ToAngle(first_z)
                            ToAngle_little(first_z)
                        start_z = first_z
                        MoveTime('b', 0.4)
                        time.sleep(0.5)
                        ###################################
                        # 定位转盘
                        global_value.set_value('model',1)
                        arm_aim_turntable()
                        adjust_zp_1(zp_x, zp_y) #定位
                        if first_z-2 < get_angle(2) < first_z+2:
                            ToAngle_little(first_z)
                        else:
                            ToAngle(first_z)
                            ToAngle_little(first_z)
                        adjust_zp_1(zp_x, zp_y) #定位
                        ###################################
                        # 打靶成品区
                        arm_cpq_1()
                        ###################################
                        # 返回暂存区
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.43)
                        MoveTime('cm', 0.7)
                        ToAngle(second_z)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        start_z = second_z
                        MoveTime('f', 2.35)
                        MoveTime('cm', 1.0)
                        ToAngle(third_z)
                        if third_z-2 < get_angle(2) < third_z+2:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        start_z = third_z
                        MoveTime('f', 1.1)
                        ###################################
                        # 定位暂存区
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_see_wl()
                        adjust_jjg_1(first_level_x, first_level_y, list(order2.keys())[1])
                        if third_z-2 < get_angle(2) < third_z+2:
                            ToAngle_little(third_z)
                        else:
                            ToAngle(third_z)
                            ToAngle_little(third_z)
                        adjust_jjg_1(first_level_x, first_level_y, list(order2.keys())[1])
                        ###################################
                        # 取二层物料
                        grab_zcq_second_level()
                        ###################################
                        # 抓完物料去精加工区
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.1)
                        MoveTime('back_zp', 1.0)
                        ToAngle(second_z)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        start_z = second_z
                        MoveTime('b', 0.9)
                        ###################################
                        # 对准精加工
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        adjust_cjg_2(cjg_x, cjg_y)
                        if second_z-2 < get_angle(2) < second_z+2:
                            ToAngle_little(second_z)
                        else:
                            ToAngle(second_z)
                            ToAngle_little(second_z)
                        adjust_cjg_2(cjg_x, cjg_y)
                        ###################################
                        # 打靶精加工
                        arm_jjg_2()
                        ###################################
                        # 打完靶后去转盘
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.3)
                        MoveTime('back_zp', 0.8)
                        ToAngle(first_z)
                        if first_z-2 < get_angle(2) < first_z+2:
                            ToAngle_little(first_z)
                        else:
                            ToAngle(first_z)
                            ToAngle_little(first_z)
                        start_z = first_z
                        MoveTime('b', 0.4)
                        time.sleep(0.5)
                        ###################################
                        # 定位转盘
                        global_value.set_value('model',1)
                        arm_aim_turntable()
                        adjust_zp_1(zp_x, zp_y) #定位
                        if first_z-2 < get_angle(2) < first_z+2:
                            ToAngle_little(first_z)
                        else:
                            ToAngle(first_z)
                            ToAngle_little(first_z)
                        adjust_zp_1(zp_x, zp_y) #定位
                        ###################################
                        # 打靶成品区
                        arm_cpq_2()
                        ###################################
                        # 回家
                        global_value.set_value('model',0)
                        MoveTime('b', 2.5)
                        MoveTime('hj_3', 1.0)
            elif a == 0:
                print('exit juesai')
                break
    elif a == 3: # 测试
        pass
    elif a == 4: # 机械臂控制
        print('arm control')
        while True:
            time.sleep(0.01)
            a, mes = Read_pm()
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2:
                arm_interim()
            elif a == 3:
                arm_losse()
            elif a == 4:
                arm_aim_turntable()
            elif a == 5:
                arm_see_wl()
            elif a == 6:
                arm_aim_bullseye()
            elif a == 7:
                S.write(data3)
            elif a == 8:
                S.write(data1)
            elif a == 9:
                S.write(data2)
            elif a == 10:
                S.write(data12)
            elif a == 11:
                S.write(data11)
            elif a == 12:
                S.write(data10)
            elif a == 13:
                S.write(data6)
            elif a == 14:
                S.write(data5)
            elif a == 15:
                S.write(data4)
            elif a == 17:
                S.write(data15)
            elif a == 18:
                S.write(data14)
            elif a == 19:
                S.write(data13)
            elif a == 16:
                print('exit arm control')
                break
    elif a == 5: # 转盘采集
        print('zp_sample')
        flag = 0
        if global_value.get_value('frame_up_flag') == 0:
            IMG_up.start()
            global_value.set_value('frame_up_flag', 1)
        while True:
            time.sleep(0.01)
            a, mes = Read_pm()
            temp_x = temp_y = 0
            if mes == b'zp':
                flag = 1
                temp = global_value.get_value('frame_up')
                temp_x, temp_y = getPos()
                pos = '"'+str(temp_x)+','+str(temp_y)+'"'
                mes = 't0.txt='+pos
                pm.write(bytearray(mes.encode()))
                pm.write(end)
            elif mes == b'wl':
                flag = 2
                temp = global_value.get_value('frame_up')
                cv2.imwrite('pic/pic_sample/wl.jpg', temp)
            elif mes == b'save':
                if flag == 1:
                    print('save zp')
                    cv2.imwrite('pic/pic_sample/zp.jpg', global_value.get_value('frame_up'))
                elif flag == 2:
                    print('save wl')
                    cv2.imwrite('pic/pic_sample/wl.jpg', global_value.get_value('frame_up'))
                elif flag == 3:
                    print('save zp_bullseye')
                    cv2.imwrite('pic/pic_sample/zp_bullseye.jpg', global_value.get_value('frame_up'))
                else:
                    print('save no')
                    pass
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2:
                flag = 3
                grab_see()
                temp = global_value.get_value('frame_up')
                cv2.imwrite('pic/pic_sample/zp_bullseye.jpg', temp)
            elif a == 3:
                arm_losse()
            elif a == 4:
                arm_aim_turntable()
            elif a == 5:
                arm_see_wl()
            elif a == 7:
                S.write(data3)
            elif a == 8:
                S.write(data1)
            elif a == 9:
                S.write(data2)
            elif a == 17:
                S.write(data15)
            elif a == 18:
                S.write(data14)
            elif a == 19:
                S.write(data13)
            elif a == 16:  #back
                print('exit zp_sample')
                break
    elif a == 6: # 打靶采集
        print('target_sample')
        flag = 0
        if global_value.get_value('frame_up_flag') == 0:
            IMG_up.start()
            global_value.set_value('frame_up_flag', 1)
        while True:
            time.sleep(0.01)
            a, mes = Read_pm()
            temp_x = temp_y = 0
            if mes == b'cjg': #cjg
                flag = 1
                temp = global_value.get_value('frame_up')
                temp_x, temp_y = getPos_2()
                pos = '"'+str(temp_x)+','+str(temp_y)+'"'
                mes = 't0.txt='+pos
                print(MoveOrStatic_2)
                pm.write(bytearray(mes.encode()))
                pm.write(end)
            elif mes == b'zcq': #zcq
                flag = 2
                temp = global_value.get_value('frame_up')
                temp_x, temp_y = getPos_2()
                pos = '"'+str(temp_x)+','+str(temp_y)+'"'
                mes = 't0.txt='+pos
                print(MoveOrStatic_2)
                pm.write(bytearray(mes.encode()))
                pm.write(end)
            elif mes == b'save':
                if flag == 1:
                    print('save cjg')
                    cv2.imwrite('pic/pic_sample/cjg.jpg', temp)
                elif flag == 2:
                    print('save zcq')
                    cv2.imwrite('pic/pic_sample/zcq1.jpg', temp)
                else:
                    pass
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2:
                arm_interim()
            elif a == 3:
                arm_losse()
            elif a == 4:
                arm_aim_turntable()
            elif a == 5:
                arm_see_wl()
            elif a == 6:
                arm_aim_bullseye()
            elif a == 7:
                S.write(data3)
            elif a == 8:
                S.write(data1)
            elif a == 9:
                S.write(data2)
            elif a == 10:
                S.write(data12)
            elif a == 11:
                S.write(data11)
            elif a == 12:
                S.write(data10)
            elif a == 13:
                S.write(data6)
            elif a == 14:
                S.write(data5)
            elif a == 15:
                S.write(data4)
            elif a == 17:
                S.write(data15)
            elif a == 18:
                S.write(data14)
            elif a == 19:
                S.write(data13)
            elif a == 16:  #back
                print('exit cjg_sample')
                break
    elif a == 7: # 码垛采集
        print('target_sample')
        flag = 0
        if global_value.get_value('frame_up_flag') == 0:
            IMG_up.start()
            global_value.set_value('frame_up_flag', 1)
        while True:
            time.sleep(0.01)
            a, mes = Read_pm()
            temp_x = temp_y = 0
            if mes == b'cjg': #cjg
                pass
            elif mes == b'zcq': #zcq
                flag = 2
                temp = global_value.get_value('frame_up')
                temp_x, temp_y = getPos_3()
                pos = '"'+str(temp_x)+','+str(temp_y)+'"'
                mes = 't0.txt='+pos
                print(MoveOrStatic_2)
                pm.write(bytearray(mes.encode()))
                pm.write(end)
            elif mes == b'save':
                if flag == 1:
                    pass
                elif flag == 2:
                    print('save zcq')
                    cv2.imwrite('pic/pic_sample/zcq2.jpg', temp)
                else:
                    pass
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2:
                arm_interim()
            elif a == 3:
                arm_losse()
            elif a == 4:
                arm_aim_turntable()
            elif a == 5:
                arm_see_wl()
            elif a == 6:
                arm_aim_bullseye()
            elif a == 7:
                S.write(data3)
            elif a == 8:
                S.write(data1)
            elif a == 9:
                S.write(data2)
            elif a == 10:
                S.write(data12)
            elif a == 11:
                S.write(data11)
            elif a == 12:
                S.write(data10)
            elif a == 13:
                S.write(data6)
            elif a == 14:
                S.write(data5)
            elif a == 15:
                S.write(data4)
            elif a == 17:
                S.write(data15)
            elif a == 18:
                S.write(data14)
            elif a == 19:
                S.write(data13)
            elif a == 16:  #back
                print('exit cjg_sample')
                break
    elif a == 8: # 决赛采集
        print('final_sample')
        flag = 0
        if global_value.get_value('frame_up_flag') == 0:
            IMG_up.start()
            global_value.set_value('frame_up_flag', 1)
        while True:
            time.sleep(0.01)
            a, mes = Read_pm()
            temp_x = temp_y = 0
            if mes == b'1c': #cjg
                flag = 1
                temp = global_value.get_value('frame_up')
                temp_x, temp_y = getPos_3()
                pos = '"'+str(temp_x)+','+str(temp_y)+'"'
                mes = 't0.txt='+pos
                print(mes)
                pm.write(bytearray(mes.encode()))
                pm.write(end)
            elif mes == b'2c': #zcq
                flag = 2
                temp = global_value.get_value('frame_up')
                temp_x, temp_y = getPos_3()
                pos = '"'+str(temp_x)+','+str(temp_y)+'"'
                mes = 't0.txt='+pos
                print(mes)
                pm.write(bytearray(mes.encode()))
                pm.write(end)
            elif mes == b'save':
                if flag == 1:
                    print('save 1_level')
                    cv2.imwrite('pic/pic_sample/1_level.jpg', temp)
                elif flag == 2:
                    print('save 2_level')
                    cv2.imwrite('pic/pic_sample/2_level.jpg', temp)
                else:
                    pass
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2: # 采集看物料顺序
                cv2.imwrite('pic/pic_sample/order.jpg', global_value.get_value('frame_up'))
            elif a == 3:
                arm_losse()
            elif a == 4:
                arm_aim_bullseye()
            elif a == 5:
                arm_see_wl()
            elif a == 6:
                arm_aim_bullseye()
            elif a == 7:
                first_level_1()
            elif a == 8:
                first_level_2()
            elif a == 9:
                first_level_3()
            elif a == 10:
                S.write(data12)
            elif a == 11:
                S.write(data11)
            elif a == 12:
                S.write(data10)
            elif a == 13:
                S.write(data6)
            elif a == 14:
                S.write(data5)
            elif a == 15:
                S.write(data4)
            elif a == 17:
                second_level_1()
            elif a == 18:
                second_level_2()
            elif a == 19:
                second_level_3()
            elif a == 16:  #back
                print('exit final_sample')
                break
