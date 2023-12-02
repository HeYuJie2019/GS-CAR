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

# GPIO14 = TXD0 -> ttyAMA0
# GPIO0  = TXD2 -> ttyAMA1
# GPIO4  = TXD3 -> ttyAMA2
# GPIO8  = TXD4 -> ttyAMA3
# GPIO12 = TXD5 -> ttyAMA4

# GPIO15 = RXD0 -> ttyAMA0
# GPIO1  = RXD2 -> ttyAMA1
# GPIO5  = RXD3 -> ttyAMA2
# GPIO9  = RXD4 -> ttyAMA3
# GPIO13 = RXD5 -> ttyAMA4
# S = serial.Serial("/dev/ttyAMA0", 9600, bytesize=8, stopbits=1, timeout=0.5) #设置机械臂串口
S = serial.Serial("/dev/ttyUSB0", 9600, bytesize=8, stopbits=1, timeout=0.5) #设置机械臂串口
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
Light_down = 2
Light_up = 3
########################设置参数#######################
start_z = 0  #初始化z轴方向
data = []    #存放扫码数据（123）
ys = []      #存放扫码数据（rgb）
FOOL = 1.05     #摩擦系数
order1 = {} #决赛半成品区顺序
order2 = {}
f = 1.0
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
template_cpq_md = cv2.imread('pic/template/template_cpq_md.jpg', cv2.IMREAD_GRAYSCALE)
template_zp_final = cv2.imread('pic/template/template_zp_final.jpg', cv2.IMREAD_GRAYSCALE)
################################################################
def OpenLight():
    GPIO.output(Light_up,1)
    GPIO.output(Light_down,1)
def CloseLight():
    GPIO.output(Light_up,0)
    GPIO.output(Light_down,0)
def OpenDownLight():
    GPIO.output(Light_up,0)
    GPIO.output(Light_down,1)
def OpenUpLight():
    GPIO.output(Light_up,1)
    GPIO.output(Light_down,0)
################################################################
#初始化引脚
def pin_init():
    GPIO.setmode(GPIO.BCM)              #select model
    GPIO_out_list = (PWMA,AIN1,AIN2,BIN1,BIN2,PWMB,PWMC,CIN1,CIN2,DIN1,DIN2,PWMD,Light_up,Light_down) #select pin
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
    Kp = 1.5
    Ki = 0.1
    Kd = 0.001
    integral = 0
    prev_error = 0

    # 设置速度上限
    max_speed = 40

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
    if abs(get_angle(2)-angle) <= 3:
        Kp = 5.0
        Ki = 0.2
        Kd = 0
    elif 6 >= abs(get_angle(2)-angle) > 3:
        Kp = 3.0
        Ki = 0.02
        Kd = 0
    elif abs(get_angle(2)-angle) > 6:
        Kp = 2.0
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
        if t2 - t1 > 1:
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
def ToAngle_Plus(target_angle):
    global_value.set_value('model', 0)
    ToAngle(target_angle)
    time.sleep(0.1)
    global_value.set_value('model', 1)
    ToAngle_little(target_angle)
    time.sleep(0.1)
    if abs(get_angle(2)-target_angle) > 0.5:
        ToAngle_adjust(target_angle)
    MoveTime('stop', 0)
def ToAngle_adjust(angle):
    global goForward_flag

    # PID控制参数
    if abs(get_angle(2)-angle) <= 1.5:
        Kp = 7.0
        Ki = 0
        Kd = 0
    elif 1.5 < abs(get_angle(2)-angle) <= 5:
        Kp = 5.0
        Ki = 0
        Kd = 0
    elif 8 >= abs(get_angle(2)-angle) > 5:
        Kp = 3.0
        Ki = 0.02
        Kd = 0
    elif abs(get_angle(2)-angle) > 8:
        Kp = 2.0
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
        if t2 - t1 > 1.2:
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
        speed_r = 78
        speed_l = 90
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed_r)
        global_value.set_value('targetB', speed_l)
        global_value.set_value('targetC', speed_l)
        global_value.set_value('targetD', speed_r)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                # print('r2')
                global_value.set_value('targetA', speed_r*1.1)
                global_value.set_value('targetB', speed_l*0.9)
                global_value.set_value('targetC', speed_l*0.9)
                global_value.set_value('targetD', speed_r*1.1)
            elif get_angle(2) - start_z < -1: #向右偏
                # print('r1')
                global_value.set_value('targetA', speed_r*1.0)
                global_value.set_value('targetB', speed_l*0.9)
                global_value.set_value('targetC', speed_l)
                global_value.set_value('targetD', speed_r)
            elif get_angle(2) - start_z > 2: #向左偏
                # print('l2')
                global_value.set_value('targetA', speed_r*0.9)
                global_value.set_value('targetB', speed_l*1.1)
                global_value.set_value('targetC', speed_l*1.1)
                global_value.set_value('targetD', speed_r*0.9)
            elif get_angle(2) - start_z > 1: #向左偏
                # print('l1')
                global_value.set_value('targetA', speed_r*0.9)
                global_value.set_value('targetB', speed_l*1.1)
                global_value.set_value('targetC', speed_l)
                global_value.set_value('targetD', speed_r)
            else:
                # print('0')
                global_value.set_value('targetA', speed_r)
                global_value.set_value('targetB', speed_l)
                global_value.set_value('targetC', speed_l)
                global_value.set_value('targetD', speed_r)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif dir == 'fast':
        move('front')
        speed_r = 180
        speed_l = 200
        t1 = t2 = time.time()
        global_value.set_value('targetA', speed_r)
        global_value.set_value('targetB', speed_l)
        global_value.set_value('targetC', speed_l)
        global_value.set_value('targetD', speed_r)
        while t2 -t1 < t:
            t2 = time.time()
            time.sleep(0.1)
            if get_angle(2) - start_z < -2: #向右偏
                # print('r2')
                global_value.set_value('targetA', speed_r*1.05)
                global_value.set_value('targetB', speed_l*0.95)
                global_value.set_value('targetC', speed_l*0.95)
                global_value.set_value('targetD', speed_r*1.05)
            elif get_angle(2) - start_z < -1: #向右偏
                # print('r1')
                global_value.set_value('targetA', speed_r*1.05)
                global_value.set_value('targetB', speed_l*0.95)
                global_value.set_value('targetC', speed_l)
                global_value.set_value('targetD', speed_r)
            elif get_angle(2) - start_z > 2: #向左偏
                # print('l2')
                global_value.set_value('targetA', speed_r*0.95)
                global_value.set_value('targetB', speed_l*1.05)
                global_value.set_value('targetC', speed_l*1.05)
                global_value.set_value('targetD', speed_r*0.95)
            elif get_angle(2) - start_z > 1: #向左偏
                # print('l1')
                global_value.set_value('targetA', speed_r*0.95)
                global_value.set_value('targetB', speed_l*1.05)
                global_value.set_value('targetC', speed_l)
                global_value.set_value('targetD', speed_r)
            else:
                # print('0')
                global_value.set_value('targetA', speed_r)
                global_value.set_value('targetB', speed_l)
                global_value.set_value('targetC', speed_l)
                global_value.set_value('targetD', speed_r)
            # print(get_angle(2),'A: ', global_value.get_value('motorA'), 'B: ', global_value.get_value('motorB'), 'C: ', global_value.get_value('motorC'), 'D: ', global_value.get_value('motorD'))
        global_value.set_value('targetA', 0)
        global_value.set_value('targetB', 0)
        global_value.set_value('targetC', 0)
        global_value.set_value('targetD', 0)
    elif dir == 'b':
        move('back')
        speed_l = 88
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
                global_value.set_value('targetA', speed_r*0.95)
                global_value.set_value('targetB', speed_l*1.05)
                global_value.set_value('targetC', speed_l*1.05)
                global_value.set_value('targetD', speed_r*0.95)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed_r)
                global_value.set_value('targetB', speed_l)
                global_value.set_value('targetC', speed_l*1.05)
                global_value.set_value('targetD', speed_r*0.95)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed_r*1.05)
                global_value.set_value('targetB', speed_l*0.95)
                global_value.set_value('targetC', speed_l*0.95)
                global_value.set_value('targetD', speed_r*1.05)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed_r)
                global_value.set_value('targetB', speed_l)
                global_value.set_value('targetC', speed_l*0.95)
                global_value.set_value('targetD', speed_r*1.05)
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
                global_value.set_value('targetA', speed*0.75)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z < -1: #向右偏
                global_value.set_value('targetA', speed)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*1.25)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 2: #向左偏
                global_value.set_value('targetA', speed*1.25)
                global_value.set_value('targetB', 0)
                global_value.set_value('targetC', speed*0.75)
                global_value.set_value('targetD', 0)
            elif get_angle(2) - start_z > 1: #向左偏
                global_value.set_value('targetA', speed*1.25)
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
################################################################
def X_decrease(num):
    motor.count_a = 0
    motor.count_c = 0
    move('front')
    global_value.set_value('targetA', 21) # 20
    global_value.set_value('targetC', 21) # 26
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
    global_value.set_value('targetA', 21)
    global_value.set_value('targetC', 21)
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
    global_value.set_value('targetB', 22)
    global_value.set_value('targetD', 22)
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
    global_value.set_value('targetB', 22)
    global_value.set_value('targetD', 22)
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
def getPos11(): # 决赛看zp
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    h_mask = cv2.inRange(h, 21, 43)
    s_mask = cv2.inRange(s, 23, 104)
    v_mask = cv2.inRange(v, 92, 208)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_zp_final, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_2(): # cjg 初赛打靶
    temp = global_value.get_value('frame_up')
    temp_hsv = cv2.cvtColor(temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    h_mask = cv2.inRange(h, 37, 60)
    s_mask = cv2.inRange(s, 25, 255)
    v_mask = cv2.inRange(v, 25, 255)
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
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 170, 180)
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
def getPos_6(color): # 转盘打靶识别物料位置
    temp = global_value.get_value('frame_up')
    height, width = temp.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=45, scale=1)
    rotated_temp = cv2.warpAffine(src=temp, M=rotate_matrix, dsize=(width, height))
    rotated_temp = rotated_temp[170:340]
    temp_hsv = cv2.cvtColor(rotated_temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 178, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
    elif color == 'g':
        h_mask = cv2.inRange(h, 30, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 77, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, template_zp_bullseye, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    x, y = maxLoc
    return x, y
def getPos_9(color): # 判断成品区物料顺序
    temp = global_value.get_value('frame_up')
    height, width = temp.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=45, scale=1)
    rotated_temp = cv2.warpAffine(src=temp, M=rotate_matrix, dsize=(width, height))
    rotated_temp = rotated_temp[180:340]
    temp_hsv = cv2.cvtColor(rotated_temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 170, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
        result = cv2.matchTemplate(mask, template_cpq_md, cv2.TM_CCOEFF_NORMED)
        # result = cv2.matchTemplate(mask, template_zp_bullseye, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        x, y = maxLoc
        return x, y, np.sum(mask)
    elif color == 'g':
        h_mask = cv2.inRange(h, 35, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 47, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_cpq_md, cv2.TM_CCOEFF_NORMED)
        # result = cv2.matchTemplate(mask, template_zp_bullseye, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        x, y = maxLoc
        return x, y, np.sum(mask)
    elif color == 'b':
        h_mask = cv2.inRange(h, 77, 124)
        s_mask = cv2.inRange(s, 20, 255)
        v_mask = cv2.inRange(v, 20, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_cpq_md, cv2.TM_CCOEFF_NORMED)
        # result = cv2.matchTemplate(mask, template_zp_bullseye, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        x, y = maxLoc
        return x, y, np.sum(mask)
def getPos_7(color): # 转盘第一次打靶识别静止或运动
    temp = global_value.get_value('frame_up')
    height, width = temp.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=45, scale=1)
    rotated_temp = cv2.warpAffine(src=temp, M=rotate_matrix, dsize=(width, height))
    rotated_temp = rotated_temp[180:-1]
    temp_hsv = cv2.cvtColor(rotated_temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 170, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h1_mask & s_mask & v_mask | h2_mask
    elif color == 'g':
        h_mask = cv2.inRange(h, 30, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 77, 124)
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
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 170, 180)
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
def getPos_10(color):# 转盘第二次打靶识别静止或运动
    temp = global_value.get_value('frame_up')
    height, width = temp.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=45, scale=1)
    rotated_temp = cv2.warpAffine(src=temp, M=rotate_matrix, dsize=(width, height))
    rotated_temp = rotated_temp[180:-1]
    temp_hsv = cv2.cvtColor(rotated_temp, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(temp_hsv)
    if color == 'r':
        h1_mask = cv2.inRange(h, 0, 15)
        h2_mask = cv2.inRange(h, 170, 180)
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
    result = cv2.matchTemplate(mask, template_cpq_md, cv2.TM_CCOEFF_NORMED)
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
def get_order_zcq(frame):
    order = {}
    r_x, _ = ColorRecognition_order_zcq('r', frame)
    b_x, _ = ColorRecognition_order_zcq('b', frame)
    g_x, _ = ColorRecognition_order_zcq('g', frame)
    if r_x < b_x and r_x < g_x:
        order['r'] = 1
    elif b_x < r_x and b_x < g_x:
        order['b'] = 1
    elif g_x < b_x and g_x < r_x:
        order['g'] = 1
    if r_x < b_x < g_x or g_x < b_x < r_x:
        order['b'] = 2
    elif b_x < r_x < g_x or g_x < r_x < b_x:
        order['r'] = 2
    elif r_x < g_x < b_x or b_x < g_x < r_x:
        order['g'] = 2
    if r_x > b_x and r_x > g_x:
        order['r'] = 3
    elif b_x > r_x and b_x > g_x:
        order['b'] = 3
    elif g_x > b_x and g_x > r_x:
        order['g'] = 3
    return order
################################################################
def get_order_cpq():
    order = {}
    r_x, _, _ = getPos_9('r')
    b_x, _, _ = getPos_9('b')
    g_x, _, _ = getPos_9('g')
    if r_x < b_x and r_x < g_x:
        order['r'] = 1
    elif b_x < r_x and b_x < g_x:
        order['b'] = 1
    elif g_x < b_x and g_x < r_x:
        order['g'] = 1
    if r_x < b_x < g_x or g_x < b_x < r_x:
        order['b'] = 2
    elif b_x < r_x < g_x or g_x < r_x < b_x:
        order['r'] = 2
    elif r_x < g_x < b_x or b_x < g_x < r_x:
        order['g'] = 2
    if r_x > b_x and r_x > g_x:
        order['r'] = 3
    elif b_x > r_x and b_x > g_x:
        order['b'] = 3
    elif g_x > b_x and g_x > r_x:
        order['g'] = 3
    return order
################################################################
def adjust_zp_1(X, Y):
    k = 0.4
    i = 1
    x, y = getPos()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-10 > x or x > X+10 or Y-10 > y or y > Y+10:
        t2 = time.time()
        x, y = getPos()
        time.sleep(0.2)
        if y > Y+10:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-10:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-10:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+10:
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
def adjust_zp_2(X, Y):
    k = 0.3
    i = 1
    x, y = getPos()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-3 > x or x > X+3 or Y-3 > y or y > Y+3:
        t2 = time.time()
        x, y = getPos()
        time.sleep(0.2)
        if y > Y+3:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-3:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-3:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+3:
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
    k = 0.35
    i = 1
    x, y = getPos_2()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-8 > x or x > X+8 or Y-8 > y or y > Y+8:
        t2 = time.time()
        x, y = getPos_2()
        time.sleep(0.2)
        if y > Y+8:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-8:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-8:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+8:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 3:
            break
        pic_name = 'pic/adjust_sample/cjg/cjg_1_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('cjg: ', getPos_2())
################################################################
def adjust_cjg_2(X, Y):
    k = 0.25
    i = 1
    x, y = getPos_2()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-3 > x or x > X+3 or Y-3 > y or y > Y+3:
        t2 = time.time()
        x, y = getPos_2()
        time.sleep(0.2)
        if y > Y+3:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-3:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-3:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+3:
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
    while  X-12 > x or x > X+12 or Y-12 > y or y > Y+12:
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
    print('zcq1: ', getPos_3())
################################################################
def adjust_zcq_2(X, Y):
    k = 0.30
    i = 1
    x, y = getPos_3()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-7 > x or x > X+7 or Y-7 > y or y > Y+7:
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
    print('zcq2: ', getPos_3())
################################################################
def adjust_order(X, Y):
    k = 0.30
    i = 1
    x, y = getPos_4()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-7 > x or x > X+7 or Y-7 > y or y > Y+7:
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
    k = 0.3
    i = 1
    x, y = getPos_5(ys)
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-5 > x or x > X+5 or Y-5 > y or y > Y+5:
        t2 = time.time()
        x, y = getPos_5(ys)
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
    k = 0.3
    i = 1
    x, y = getPos_8(ys)
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-5 > x or x > X+5 or Y-5 > y or y > Y+5:
        t2 = time.time()
        x, y = getPos_8(ys)
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
        pic_name = 'pic/adjust_sample/jjg2/jjg_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('jjg: ', getPos_8(ys))
################################################################
def adjust_zp_final(X, Y):
    k = 0.3
    i = 1
    x, y = getPos11()
    t1 = t2 = time.time()
    global_value.set_value('model', 1)
    while  X-3 > x or x > X+3 or Y-3 > y or y > Y+3:
        t2 = time.time()
        x, y = getPos11()
        time.sleep(0.2)
        if y > Y+3:
            Y_decrease((y-Y)*k)
            time.sleep(0.2)
        if y < Y-3:
            Y_increase((Y-y)*k)
            time.sleep(0.2)
        if x < X-3:
            X_increase((X-x)*k)
            time.sleep(0.2)
        if x > X+3:
            X_decrease((x-X)*k)
            time.sleep(0.2)
        if t2 - t1 > 5:
            break
        pic_name = 'pic/adjust_sample/zp_final/zp_' + str(i) +'.jpg'
        i += 1
        cv2.imwrite(pic_name, global_value.get_value('frame_up'))
    global_value.set_value('targetA', 0)
    global_value.set_value('targetB', 0)
    global_value.set_value('targetC', 0)
    global_value.set_value('targetD', 0)
    print('zp: ', getPos())
################################################################
def red():
    S.write(bytes.fromhex('ff 01 07 28 00'))
    S.write(bytes.fromhex('ff 02 07 6e 02'))
def green():
    S.write(bytes.fromhex('ff 01 07 28 00'))
    S.write(bytes.fromhex('ff 02 07 56 05'))
def blue():
    S.write(bytes.fromhex('ff 01 07 28 00'))
    S.write(bytes.fromhex('ff 02 07 6b 08'))
# 折叠
def arm_fold():
    S.write(bytes.fromhex('ff 01 09 0a 00'))
    S.write(bytes.fromhex('ff 01 0b 0a 00'))
    S.write(bytes.fromhex('ff 02 09 97 09'))
    S.write(bytes.fromhex('ff 02 0b 09 03'))
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
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 57 04'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 98 07'))
    S.write(bytes.fromhex('ff 01 08 28 00'))
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
    S.write(bytes.fromhex('ff 02 0b fd 05'))
# 放手动作
def arm_losses():
    S.write(bytes.fromhex('ff 01 0b 14 00'))
    S.write(bytes.fromhex('ff 02 0b 8e 04'))
def arm_interim():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 af 06'))
    S.write(bytes.fromhex('ff 01 09 0c 00'))
    S.write(bytes.fromhex('ff 02 09 6c 05'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 78 04'))
    S.write(bytes.fromhex('ff 02 0b dc 03'))
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
    S.write(bytes.fromhex('ff 02 08 52 02')) 
    time.sleep(0.5)
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 09 fd 04'))
    S.write(bytes.fromhex('ff 02 0a 60 07'))
    S.write(bytes.fromhex('ff 02 0b c6 04'))
def first_level_2():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 41 03'))
    time.sleep(0.5)
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 08 41 03'))
    S.write(bytes.fromhex('ff 02 09 6c 05'))
    S.write(bytes.fromhex('ff 02 0a 8d 07'))
    S.write(bytes.fromhex('ff 02 0b c6 04'))
def first_level_3():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 14 04'))
    time.sleep(0.5)
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 01 0a 14 00'))
    S.write(bytes.fromhex('ff 02 08 14 04'))
    S.write(bytes.fromhex('ff 02 09 fd 04'))
    S.write(bytes.fromhex('ff 02 0a 55 07'))
    S.write(bytes.fromhex('ff 02 0b c6 04'))
# 二层抓取
def second_level_1():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 52 02')) 
    time.sleep(0.5)
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 1f 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a fd 05'))
    S.write(bytes.fromhex('ff 02 0b 8e 04'))
def second_level_2():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 41 03'))
    time.sleep(0.5)
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 b0 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 4b 06'))
    S.write(bytes.fromhex('ff 02 0b 8e 04'))
def second_level_3():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 c6 03'))
    time.sleep(0.5)
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 1f 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a fd 05'))
    S.write(bytes.fromhex('ff 02 0b 8e 04'))
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
# 转盘抓取一
def arm_zp1():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 20 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 a4 06'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a 4b 06'))
# 转盘抓取二
def arm_zp2():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 c7 02'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 40 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
# 转盘抓取三
def arm_zp3():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 02 08 84 03'))
    S.write(bytes.fromhex('ff 01 09 18 00'))
    S.write(bytes.fromhex('ff 02 09 57 04'))
    S.write(bytes.fromhex('ff 01 0a 18 00'))
    S.write(bytes.fromhex('ff 02 0a dc 05'))
# 转盘码垛一
def arm_zp_md1():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 01 0a 0a 00'))
    S.write(bytes.fromhex('ff 02 0a dc 04'))
    S.write(bytes.fromhex('ff 02 09 d0 06'))
    S.write(bytes.fromhex('ff 02 08 2b 03'))
# 转盘码垛二
def arm_zp_md2():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 01 0a 0a 00'))
    S.write(bytes.fromhex('ff 02 08 bc 02'))
    S.write(bytes.fromhex('ff 02 09 dc 05'))
    S.write(bytes.fromhex('ff 02 0a fd 04'))
# 转盘码垛三
def arm_zp_md3():
    S.write(bytes.fromhex('ff 01 08 18 00'))
    S.write(bytes.fromhex('ff 01 09 14 00'))
    S.write(bytes.fromhex('ff 01 0a 0a 00'))
    S.write(bytes.fromhex('ff 02 08 b0 03'))
    S.write(bytes.fromhex('ff 02 09 dc 05'))
    S.write(bytes.fromhex('ff 02 0a fd 04'))
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
    height, width = img.shape[:2]
    center = (width/2, height/2)
    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=15, scale=1)
    rotated_temp = cv2.warpAffine(src=img, M=rotate_matrix, dsize=(width, height))
    img_hsv = cv2.cvtColor(rotated_temp, cv2.COLOR_BGR2HSV)
    if color == 'b':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 100, 124)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_wl, cv2.TM_CCOEFF_NORMED)
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
        result = cv2.matchTemplate(mask, template_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
    elif color == 'g':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 37, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
    else:
        (startX, startY) = (0, 0)
    return (startX, startY)
def ColorRecognition_order_zcq(color, img):
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
        h_mask = cv2.inRange(h, 37, 77)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, template_order_2, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
    else:
        (startX, startY) = (0, 0)
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
    cv2.imwrite('pic/color_sample/sample_zp_order_1'+'.jpg', frame)
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
        time.sleep(3.3)
    elif ys[0] == 'b':
        S.write(data15)
        time.sleep(3.3)
    #######第2个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[1]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    order_temp = get_order(frame)
    cv2.imwrite('pic/color_sample/sample_zp_order_2'+'.jpg', frame)
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
        time.sleep(3.3)
    elif ys[1] == 'b':
        S.write(data15)
        time.sleep(3.3)
    #######第3个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[2]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    cv2.imwrite('pic/color_sample/sample_zp_order_3'+'.jpg', frame)
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
        time.sleep(3.3)
    elif ys[2] == 'g':
        S.write(data14)
        time.sleep(3.3)
    elif ys[2] == 'b':
        S.write(data15)
        time.sleep(3.3)
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
    cv2.imwrite('pic/color_sample/sample_zp_order_4'+'.jpg', frame)
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
        time.sleep(3.3)
    elif ys[3] == 'b':
        S.write(data15)
        time.sleep(3.3)
    #######第2个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[4]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    cv2.imwrite('pic/color_sample/sample_zp_order_5'+'.jpg', frame)
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
        time.sleep(3.3)
    elif ys[4] == 'b':
        S.write(data15)
        time.sleep(3.3)
    #######第3个物料#######
    time.sleep(0.5)
    while MoveOrStatic(ys[5]) == 0:
        time.sleep(0.1)
        frame = global_value.get_value('frame_up')
    frame = global_value.get_value('frame_up')
    cv2.imwrite('pic/color_sample/sample_zp_order_6'+'.jpg', frame)
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
        time.sleep(3.3)
    elif ys[5] == 'g':
        S.write(data14)
        time.sleep(3.3)
    elif ys[5] == 'b':
        S.write(data15)
        time.sleep(3.3)
    arm_initialize()
################################################################
# 初赛第一次粗加工打靶并抓取
def arm_cjg():
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[0] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[0] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[1] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[1] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[2] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[2] == 'b':
        S.write(data10)
        time.sleep(3.9)
        
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data7)
        time.sleep(4.1)
    elif ys[0] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[0] == 'b':
        S.write(data9)
        time.sleep(3.8)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data7)
        time.sleep(4.1)
    elif ys[1] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[1] == 'b':
        S.write(data9)
        time.sleep(3.8)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data7)
        time.sleep(4.1)
    elif ys[2] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[2] == 'b':
        S.write(data9)
        time.sleep(3.8)
    arm_initialize()  
# 初赛第二次粗加工打靶并抓取
def arm_cjg_2():
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[3] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[3] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[4] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[4] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[5] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[5] == 'b':
        S.write(data10)
        time.sleep(3.9)
        
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data7)
        time.sleep(4.1)
    elif ys[3] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[3] == 'b':
        S.write(data9)
        time.sleep(3.8)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data7)
        time.sleep(4.1)
    elif ys[4] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[4] == 'b':
        S.write(data9)
        time.sleep(3.8)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data7)
        time.sleep(4.1)
    elif ys[5] == 'g':
        S.write(data8)
        time.sleep(3.8)
    elif ys[5] == 'b':
        S.write(data9)
        time.sleep(3.8)
    arm_initialize()  
################################################################
# 初赛第一次暂存区打靶
def arm_zcq():
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[0] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[0] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[1] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[1] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[2] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[2] == 'b':
        S.write(data10)
        time.sleep(3.9)
    arm_initialize()
# 初赛第二次暂存区打靶
def arm_zcq_2():
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data6)
        time.sleep(3.9)
    elif ys[3] == 'g':
        S.write(data5)
        time.sleep(3.7)
    elif ys[3] == 'b':
        S.write(data4)
        time.sleep(3.5)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data6)
        time.sleep(3.9)
    elif ys[4] == 'g':
        S.write(data5)
        time.sleep(3.7)
    elif ys[4] == 'b':
        S.write(data4)
        time.sleep(3.5)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data6)
        time.sleep(3.9)
    elif ys[5] == 'g':
        S.write(data5)
        time.sleep(3.7)
    elif ys[5] == 'b':
        S.write(data4)
        time.sleep(3.5)
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
    if ys[0] == 'b':  
        blue()
        S.write(data3)
        time.sleep(3.4)
    elif ys[0] == 'g':
        green()
        S.write(data3)
        time.sleep(3.4)
    elif ys[0] == 'r':
        red()
        S.write(data3)
        time.sleep(3.4)
        
    time.sleep(0.5)
        
    if order1[ys[1]] == 1:
        first_level_1()
    elif order1[ys[1]] == 2:
        first_level_2()
    elif order1[ys[1]] == 3:
        first_level_3()
    time.sleep(0.5)
    if ys[1] == 'b':  
        blue()
        S.write(data3)
        time.sleep(3.4)
    elif ys[1] == 'g':
        green()
        S.write(data3)
        time.sleep(3.4)
    elif ys[1] == 'r':
        red()
        S.write(data3)
        time.sleep(3.4)
    
    time.sleep(0.5)

    if order1[ys[2]] == 1:
        first_level_1()
    elif order1[ys[2]] == 2:
        first_level_2()
    elif order1[ys[2]] == 3:
        first_level_3()
    time.sleep(0.5)
    if ys[2] == 'b':  
        blue()
        S.write(data3)
        time.sleep(3.4)
    elif ys[2] == 'g':
        green()
        S.write(data3)
        time.sleep(3.4)
    elif ys[2] == 'r':
        red()
        S.write(data3)
        time.sleep(3.4)
    arm_initialize()
# 决赛抓取第二层物料
def grab_zcq_second_level():
    if order2[ys[3]] == 1:
        second_level_1()
    elif order2[ys[3]] == 2:
        second_level_2()
    elif order2[ys[3]] == 3:
        second_level_3()
    time.sleep(0.8)
    if ys[3] == 'b':  
        blue()
        S.write(data3)
        time.sleep(3.4)
    elif ys[3] == 'g':
        green()
        S.write(data3)
        time.sleep(3.4)
    elif ys[3] == 'r':
        red()
        S.write(data3)
        time.sleep(3.4)
    
    time.sleep(0.8)
        
    if order2[ys[4]] == 1:
        second_level_1()
    elif order2[ys[4]] == 2:
        second_level_2()
    elif order2[ys[4]] == 3:
        second_level_3()
    time.sleep(0.8)
    if ys[4] == 'b':  
        blue()
        S.write(data3)
        time.sleep(3.4)
    elif ys[4] == 'g':
        green()
        S.write(data3)
        time.sleep(3.4)
    elif ys[4] == 'r':
        red()
        S.write(data3)
        time.sleep(3.4)

    time.sleep(0.8)
    
    if order2[ys[5]] == 1:
        second_level_1()
    elif order2[ys[5]] == 2:
        second_level_2()
    elif order2[ys[5]] == 3:
        second_level_3()
    time.sleep(0.5)
    if ys[5] == 'b':  
        blue()
        S.write(data3)
        time.sleep(3.4)
    elif ys[5] == 'g':
        green()
        S.write(data3)
        time.sleep(3.4)
    elif ys[5] == 'r':
        red()
        S.write(data3)
        time.sleep(3.4)
    arm_initialize()
################################################################
# 决赛第一次打靶精加工区域(rgb)
def arm_jjg_1():
    #######第1个物料#######
    if ys[0] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[0] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[0] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第2个物料#######
    if ys[1] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[1] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[1] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第3个物料#######
    if ys[2] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[2] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[2] == 'b':
        S.write(data10)
        time.sleep(3.9)
    
    #######第1个物料#######
    if ys[0] == 'b':
        S.write(data4)
        time.sleep(4.0)
    elif ys[0] == 'g':
        S.write(data5)
        time.sleep(4.0)
    elif ys[0] == 'r':
        S.write(data6)
        time.sleep(4.3)
    #######第2个物料#######
    if ys[1] == 'b':
        S.write(data4)
        time.sleep(4.0)
    elif ys[1] == 'g':
        S.write(data5)
        time.sleep(4.0)
    elif ys[1] == 'r':
        S.write(data6)
        time.sleep(4.3)
    #######第3个物料#######
    if ys[2] == 'b':
        S.write(data4)
        time.sleep(4.0)
    elif ys[2] == 'g':
        S.write(data5)
        time.sleep(4.0)
    elif ys[2] == 'r':
        S.write(data6)
        time.sleep(4.3)
    arm_initialize()  
# 决赛第二次打靶精加工区域(rgb)
def arm_jjg_2():
    #######第1个物料#######
    if ys[3] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[3] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[3] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第2个物料#######
    if ys[4] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[4] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[4] == 'b':
        S.write(data10)
        time.sleep(3.9)
    #######第3个物料#######
    if ys[5] == 'r':
        S.write(data12)
        time.sleep(4.45)
    elif ys[5] == 'g':
        S.write(data11)
        time.sleep(4.2)
    elif ys[5] == 'b':
        S.write(data10)
        time.sleep(3.9)
    
    #######第1个物料#######
    if ys[3] == 'b':
        S.write(data4)
        time.sleep(4.0)
    elif ys[3] == 'g':
        S.write(data5)
        time.sleep(4.0)
    elif ys[3] == 'r':
        S.write(data6)
        time.sleep(4.3)
    #######第2个物料#######
    if ys[4] == 'b':
        S.write(data4)
        time.sleep(4.0)
    elif ys[4] == 'g':
        S.write(data5)
        time.sleep(4.0)
    elif ys[4] == 'r':
        S.write(data6)
        time.sleep(4.3)
    #######第3个物料#######
    if ys[5] == 'b':
        S.write(data4)
        time.sleep(4.0)
    elif ys[5] == 'g':
        S.write(data5)
        time.sleep(4.0)
    elif ys[5] == 'r':
        S.write(data6)
        time.sleep(4.3)
    arm_initialize()  
################################################################
# 判断转盘是运动还是静止
def MoveOrStatic(color):
    frame1 = global_value.get_value('frame_up')
    x1, y1 = ColorRecognition(color, frame1)
    time.sleep(0.1)
    frame2 = global_value.get_value('frame_up')
    x2, y2 = ColorRecognition(color, frame2)
    # return abs(x2-x1) + abs(y2-y1)
    if abs(x2-x1) + abs(y2-y1) >= 3:
        return 0   #move
    elif abs(x2-x1) + abs(y2-y1) < 3:
        return 1   #static
def MoveOrStatic_2(color):# 成品区一次打靶判断静止或移动
    x1, y1 = getPos_7(color)
    time.sleep(0.1)
    x2, y2 = getPos_7(color)
    if abs(x2-x1) + abs(y2-y1) >= 2:
        return 'move'   #move
    elif abs(x2-x1) + abs(y2-y1) < 2:
        return 'static'   #static
def MoveOrStatic_3(color): # 成品区二次打靶判断静止或移动
    x1, y1 = getPos_10(color)
    time.sleep(0.1)
    x2, y2 = getPos_10(color)
    if abs(x2-x1) + abs(y2-y1) >= 2:
        return 'move'   #move
    elif abs(x2-x1) + abs(y2-y1) < 2:
        return 'static'   #static
################################################################
def arm_cpq_1():
    if ys[0] == 'b':
        S.write(data15)
        time.sleep(2.9)
    elif ys[0] == 'g':
        S.write(data14)
        time.sleep(2.9)
    elif ys[0] == 'r':
        S.write(data13)
        time.sleep(2.9)
    while 1:
        if MoveOrStatic_2(ys[0]) == 'static':
            break
    cv2.imwrite('pic/adjust_sample/cpq/cpq1-1.jpg', global_value.get_value('frame_up'))
    order_cpq = get_order_cpq()
    if order_cpq[ys[0]] == 1:
        S.write(data7)
        time.sleep(1.3)
    elif order_cpq[ys[0]] == 2:
        S.write(data8)
        time.sleep(1.3)
    elif order_cpq[ys[0]] == 3:
        S.write(data9)
        time.sleep(1.3)
        
        
    if ys[1] == 'b':
        S.write(data15)
        time.sleep(2.9)
    elif ys[1] == 'g':
        S.write(data14)
        time.sleep(2.9)
    elif ys[1] == 'r':
        S.write(data13)
        time.sleep(2.9)
    while 1:
        if MoveOrStatic_2(ys[1]) == 'static':
            break
    cv2.imwrite('pic/adjust_sample/cpq/cpq1-2.jpg', global_value.get_value('frame_up'))
    order_cpq = get_order_cpq()
    if order_cpq[ys[1]] == 1:
        S.write(data7)
        time.sleep(1.3)
    elif order_cpq[ys[1]] == 2:
        S.write(data8)
        time.sleep(1.3)
    elif order_cpq[ys[1]] == 3:
        S.write(data9)
        time.sleep(1.3)
        
        
    if ys[2] == 'b':
        S.write(data15)
        time.sleep(2.9)
    elif ys[2] == 'g':
        S.write(data14)
        time.sleep(2.9)
    elif ys[2] == 'r':
        S.write(data13)
        time.sleep(2.9)
    while 1:
        if MoveOrStatic_2(ys[2]) == 'static':
            break
    cv2.imwrite('pic/adjust_sample/cpq/cpq1-3.jpg', global_value.get_value('frame_up'))
    order_cpq = get_order_cpq()
    if order_cpq[ys[2]] == 1:
        S.write(data7)
        time.sleep(1.3)
    elif order_cpq[ys[2]] == 2:
        S.write(data8)
        time.sleep(1.3)
    elif order_cpq[ys[2]] == 3:
        S.write(data9)
        time.sleep(1.3)
    arm_initialize()
def arm_cpq_2():
    if ys[3] == 'b':
        S.write(data15)
        time.sleep(2.9)
    elif ys[3] == 'g':
        S.write(data14)
        time.sleep(2.9)
    elif ys[3] == 'r':
        S.write(data13)
        time.sleep(2.9)
    while MoveOrStatic_3(ys[3]) == 'move' or (get_order_cpq()[ys[3]] == 3 and getPos_9(ys[3])[2]>1000000):
        pass
    cv2.imwrite('pic/adjust_sample/cpq/cpq2-1.jpg', global_value.get_value('frame_up'))
    order_cpq = get_order_cpq()
    if order_cpq[ys[3]] == 1:
        if getPos_9(ys[3])[2] < 1500000:
            S.write(data7)
            time.sleep(1.3)
        else:
            arm_zp_md2()
            time.sleep(1)
            arm_losses()
            time.sleep(0.3)
    elif order_cpq[ys[3]] == 2:
        if getPos_9(ys[3])[2] < 1500000:
            S.write(data8)
            time.sleep(1.3)
        else:
            arm_zp_md1()
            time.sleep(1)
            arm_losses()
            time.sleep(0.3)
    elif order_cpq[ys[3]] == 3:
        S.write(data9)
        time.sleep(1.3)
        
        
    if ys[4] == 'b':
        S.write(data15)
        time.sleep(2.9)
    elif ys[4] == 'g':
        S.write(data14)
        time.sleep(2.9)
    elif ys[4] == 'r':
        S.write(data13)
        time.sleep(2.9)
    while MoveOrStatic_3(ys[3]) == 'move' or (get_order_cpq()[ys[4]] == 3 and getPos_9(ys[4])[2]>1000000):
        pass
    cv2.imwrite('pic/adjust_sample/cpq/cpq2-2.jpg', global_value.get_value('frame_up'))
    order_cpq = get_order_cpq()
    if order_cpq[ys[4]] == 1:
        if getPos_9(ys[4])[2] < 1500000:
            S.write(data7)
            time.sleep(1.3)
        else:
            arm_zp_md2()
            time.sleep(0.5)
            arm_losses()
            time.sleep(0.3)
    elif order_cpq[ys[4]] == 2:
        if getPos_9(ys[4])[2] < 1500000:
            S.write(data8)
            time.sleep(1.3)
        else:
            arm_zp_md1()
            time.sleep(0.5)
            arm_losses()
            time.sleep(0.3)
    elif order_cpq[ys[4]] == 3:
        S.write(data9)
        time.sleep(1.3)
        
        
    if ys[5] == 'b':
        S.write(data15)
        time.sleep(2.9)
    elif ys[5] == 'g':
        S.write(data14)
        time.sleep(2.9)
    elif ys[5] == 'r':
        S.write(data13)
        time.sleep(2.9)
    while MoveOrStatic_3(ys[5]) == 'move' or (get_order_cpq()[ys[5]] == 3 and getPos_9(ys[5])[2]>1000000):
        pass
    cv2.imwrite('pic/adjust_sample/cpq/cpq2-3.jpg', global_value.get_value('frame_up'))
    order_cpq = get_order_cpq()
    if order_cpq[ys[5]] == 1:
        if getPos_9(ys[5])[2] < 1500000:
            S.write(data7)
            time.sleep(1.3)
        else:
            arm_zp_md2()
            time.sleep(0.5)
            arm_losses()
            time.sleep(0.3)
    elif order_cpq[ys[5]] == 2:
        if getPos_9(ys[5])[2] < 1500000:
            S.write(data8)
            time.sleep(1.3)
        else:
            arm_zp_md1()
            time.sleep(0.5)
            arm_losses()
            time.sleep(0.3)
    elif order_cpq[ys[5]] == 3:
        S.write(data9)
        time.sleep(1.3)
    time.sleep(0.5)
    arm_interim()
################################################################
# 定义进程
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
h_mask = cv2.inRange(h, 21, 35)
s_mask = cv2.inRange(s, 23, 255)
v_mask = cv2.inRange(v, 23, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_zp, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
zp_x, zp_y = maxLoc
print('zp: ', maxLoc)
################################################################
zp_final = cv2.imread('pic/pic_sample/zp_final.jpg')
temp_hsv = cv2.cvtColor(zp_final, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 21, 43)
s_mask = cv2.inRange(s, 23, 104)
v_mask = cv2.inRange(v, 92, 208)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_zp_final, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
zp_final_x, zp_final_y = maxLoc
zp_final_x, zp_final_y = zp_final_x, zp_final_y-5
print('zp_final: ', maxLoc)
################################################################
cjg = cv2.imread('pic/pic_sample/cjg.jpg')
temp_hsv = cv2.cvtColor(cjg, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 37, 60)
s_mask = cv2.inRange(s, 25, 255)
v_mask = cv2.inRange(v, 25, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_cjg, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
cjg_x, cjg_y = maxLoc
print('cjg: ', maxLoc)
################################################################
cjg = cv2.imread('pic/pic_sample/jjg.jpg')
temp_hsv = cv2.cvtColor(cjg, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 37, 60)
s_mask = cv2.inRange(s, 25, 255)
v_mask = cv2.inRange(v, 25, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_cjg, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
jjg_x, jjg_y = maxLoc
print('jjg: ', maxLoc)
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
print('zcq1: ', zcq1_x, zcq1_y)
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
first_level = cv2.imread('pic/pic_sample/1_level_g.jpg')
temp_hsv = cv2.cvtColor(first_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 56, 77)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_1_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
first_level_x_g, first_level_y_g = maxLoc
print('first_level_g: ', maxLoc)
################################################################
first_level = cv2.imread('pic/pic_sample/1_level_b.jpg')
temp_hsv = cv2.cvtColor(first_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 100, 124)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_1_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
first_level_x_b, first_level_y_b = maxLoc
print('first_level_b: ', maxLoc)
################################################################
first_level = cv2.imread('pic/pic_sample/1_level_r.jpg')
temp_hsv = cv2.cvtColor(first_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h1_mask = cv2.inRange(h, 0, 15)
h2_mask = cv2.inRange(h, 170, 180)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h1_mask & s_mask & v_mask | h2_mask
result = cv2.matchTemplate(mask, template_1_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
first_level_x_r, first_level_y_r = maxLoc
print('first_level_r: ', maxLoc)
################################################################
second_level = cv2.imread('pic/pic_sample/2_level_g.jpg')
temp_hsv = cv2.cvtColor(second_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 56, 77)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_2_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
second_level_x_g, second_level_y_g = maxLoc
print('second_level_g: ', maxLoc)
################################################################
second_level = cv2.imread('pic/pic_sample/2_level_b.jpg')
temp_hsv = cv2.cvtColor(second_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h_mask = cv2.inRange(h, 100, 124)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h_mask & s_mask & v_mask
result = cv2.matchTemplate(mask, template_2_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
second_level_x_b, second_level_y_b = maxLoc
print('second_level_b: ', maxLoc)
################################################################
second_level = cv2.imread('pic/pic_sample/2_level_r.jpg')
temp_hsv = cv2.cvtColor(second_level, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(temp_hsv)
h1_mask = cv2.inRange(h, 0, 15)
h2_mask = cv2.inRange(h, 170, 180)
s_mask = cv2.inRange(s, 43, 255)
v_mask = cv2.inRange(v, 46, 255)
mask = h1_mask & s_mask & v_mask | h2_mask
result = cv2.matchTemplate(mask, template_2_level, cv2.TM_CCOEFF_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
second_level_x_r, second_level_y_r = maxLoc
print('second_level_r: ', maxLoc)
################################################################
pin_init()
print('ready')
CloseLight()
Page_pm(0)
_ = pm.read_all()
while True:
    a, mes = Read_pm()
    time.sleep(0.01)
    if a == 1: # 初赛
        # IMG_up.start()
        print('chu sai')
        arm_fold()
        while True:
            a, mes = Read_pm()
            time.sleep(0.01)
            if a == 2:
                CloseLight()
                print('MCU write zero')
                # print(getPos_2())
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
                Page_pm(9)
                while True:
                    a, mes = Read_pm()
                    time.sleep(0.01)
                    if a == 1:
                        # 正式开始
                        S.write(data0)
                        global_value.set_value('model', 1)
                        MoveTime('cm', 0.9*f)
                        global_value.set_value('model', 0)
                        arm_initialize()
                        MoveTime('f', 0.8*f)
                        OpenDownLight()
                        while len(ys) == 0:
                            if cap_temp.isOpened():
                                print('down video is work')
                                ret, frame = cap_temp.read()
                                if np.sum(frame) == 0:
                                    print('fuck!!!')
                                    break
                                if ret:
                                    get_qr_data(frame)
                        if len(data) != 0:
                            for i in range(3):
                                display_num(i, int(data[i]))
                            for i in range(3,6):
                                display_num(i, int(data[i+1]))
                        cap_temp.release() # 关闭底部摄像头
                        CloseLight()
                        ###################################
                        # 定位转盘
                        IMG_up.start()
                        global_value.set_value('frame_up_flag', 1)
                        MoveTime('f', 1.4*f)
                        OpenLight()
                        time.sleep(0.5)
                        arm_aim_turntable()
                        print(ys)
                        ToAngle_adjust(first_z)
                        adjust_zp_1(zp_x, zp_y) # 定位
                        if abs(first_z - get_angle(2)) > 2:
                            ToAngle_adjust(first_z)
                            adjust_zp_1(zp_x, zp_y) # 定位
                        if abs(first_z - get_angle(2)) > 2:
                            ToAngle_adjust(first_z)
                            adjust_zp_1(zp_x, zp_y) # 定位
                        if abs(first_z - get_angle(2)) > 2:
                            ToAngle_adjust(first_z)
                            adjust_zp_1(zp_x, zp_y) # 定位
                        ###################################
                        # 抓物料
                        arm_see_wl()
                        time.sleep(1)
                        grab_zp()
                        ###################################
                        CloseLight()
                        # 抓完物料定位粗加工区
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.40*f)
                        MoveTime('cm', 0.7*f)
                        ToAngle_Plus(second_z)
                        global_value.set_value('model',0)
                        start_z = second_z
                        MoveTime('f', 1.30*f)
                        OpenLight()
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        ToAngle_adjust(second_z)
                        adjust_cjg_1(cjg_x, cjg_y)
                        ToAngle_adjust(second_z)
                        adjust_cjg_2(cjg_x, cjg_y)
                        if abs(second_z - get_angle(2)) > 2:
                            ToAngle_adjust(second_z)
                            adjust_cjg_2(cjg_x, cjg_y)
                        CloseLight()
                        ###################################
                        # 打靶粗加工
                        arm_cjg()
                        ###################################
                        # 打完靶后定位暂存区
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.9*f)
                        MoveTime('cm', 1.0*f)
                        ToAngle_Plus(third_z)
                        global_value.set_value('model', 0)
                        start_z = third_z
                        MoveTime('f', 1.23*f)
                        OpenLight()
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        ToAngle_adjust(third_z)
                        adjust_cjg_1(zcq1_x, zcq1_y)
                        ToAngle_adjust(third_z)
                        adjust_cjg_2(zcq1_x, zcq1_y)
                        if abs(third_z - get_angle(2)) > 2:
                            ToAngle_adjust(third_z)
                            adjust_cjg_2(cjg_x, cjg_y)
                        CloseLight()
                        ###################################
                        # 打靶暂存区
                        arm_zcq()
                        ###################################
                        # 放置好物料后返回转盘
                        i_flag = 2
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.1*f)
                        MoveTime('back_zp', 1.0*f)
                        ToAngle_Plus(second_z)
                        global_value.set_value('model', 0)
                        start_z = second_z
                        MoveTime('b', 2.3*f)
                        MoveTime('back_zp', 1.0*f)
                        ToAngle_Plus(first_z)
                        global_value.set_value('model', 0)
                        ###################################修正###################################
                        start_z = first_z-0.5  
                        MoveTime('b', 0.4*f)
                        OpenLight()
                        time.sleep(0.5)
                        ###################################
                        # 定位转盘
                        global_value.set_value('model',1)
                        arm_aim_turntable()
                        ToAngle_adjust(start_z)
                        adjust_zp_1(zp_x, zp_y) #定位
                        if abs(start_z - get_angle(2)) > 2:
                            ToAngle_adjust(start_z)
                            adjust_zp_1(zp_x, zp_y) #定位
                        CloseLight()
                        ###################################
                        # 抓取第二波物料
                        arm_see_wl()
                        time.sleep(1)
                        grab_zp_2()
                        ###################################
                        # 定位粗加工区
                        arm_initialize()
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.43*f)
                        MoveTime('cm', 0.8*f)
                        ###################################修正###################################
                        start_z = second_z-1
                        ToAngle_Plus(start_z)
                        global_value.set_value('model', 0)
                        MoveTime('f', 1.30*f)
                        OpenLight()
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        ToAngle_adjust(start_z)
                        adjust_cjg_1(cjg_x, cjg_y)
                        ToAngle_adjust(start_z)
                        adjust_cjg_2(cjg_x, cjg_y)
                        if abs(start_z - get_angle(2)) > 2:
                            ToAngle_adjust(start_z)
                            adjust_cjg_2(cjg_x, cjg_y)
                        if abs(start_z - get_angle(2)) > 2:
                            ToAngle_adjust(start_z)
                            adjust_cjg_1(cjg_x, cjg_y)
                        CloseLight()
                        ###################################
                        # 打靶
                        arm_cjg_2()
                        ###################################
                        # 定位暂存区
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.9*f)
                        MoveTime('cm', 1.0*f)
                        start_z = third_z-1.5
                        ###################################修正###################################
                        ToAngle_Plus(start_z)
                        global_value.set_value('model', 0)
                        MoveTime('f', 1.20*f)
                        OpenLight()
                        time.sleep(0.5)
                        global_value.set_value('model', 1)
                        arm_aim_bullseye()
                        ToAngle_adjust(start_z)
                        adjust_zcq_2(zcq2_x, zcq2_y)
                        if abs(start_z - get_angle(2)) > 2:
                            ToAngle_adjust(start_z)
                            adjust_zcq_2(zcq2_x, zcq2_y)
                        CloseLight()
                        ###################################
                        # 打靶
                        arm_zcq_2()
                        ###################################
                        global_value.set_value('model', 0)
                        MoveTime('f', 1.40*f)
                        MoveTime('hj_1', 0.8*f)
                        ###################################修正###################################
                        start_z = fouth_z-2
                        ToAngle_Plus(start_z)
                        global_value.set_value('model', 0)
                        
                        MoveTime('f', 2.75*f)
                        MoveTime('hj_2', 1.0*f)
            elif a == 0:
                print('exit chusai')
                break
    elif a == 2: # 决赛    
        print('jue sai')
        arm_fold()
        while True:
            a, mes = Read_pm()
            time.sleep(0.01)
            if a == 2:
                CloseLight()
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
                Page_pm(9)
                while True:
                    a, mes = Read_pm()
                    time.sleep(0.01)
                    if a == 1:
                        S.write(data1)
                        # 正式开始
                        global_value.set_value('model', 1)
                        MoveTime('cm', 1.0*f)
                        global_value.set_value('model', 0)
                        arm_initialize()
                        MoveTime('f', 0.73*f)
                        OpenDownLight()
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
                        CloseLight()
                        ###################################
                        # 去暂存区
                        IMG_up.start()
                        global_value.set_value('frame_up_flag', 1)
                        MoveTime('b', 0.73*f)
                        MoveTime('back_zp', 0.8*f)
                        ToAngle_Plus(second_z)
                        if abs(get_angle(2)-second_z) > 0.5:
                            ToAngle_adjust(second_z)
                        global_value.set_value('model', 0)
                        start_z = second_z
                        MoveTime('f', 2.19*f)
                        MoveTime('hj_2', 1.2*f)
                        ToAngle_Plus(third_z)
                        global_value.set_value('model', 0)
                        start_z = third_z
                        OpenLight()
                        MoveTime('b', 1.35*f)
                        arm_see_wl()
                        time.sleep(1)
                        ###################################
                        # 看物料顺序
                        Order_temp = global_value.get_value('frame_up')
                        height, width = Order_temp.shape[:2]
                        center = (width/2, height/2)
                        rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=45, scale=1)
                        rotated_temp = cv2.warpAffine(src=Order_temp, M=rotate_matrix, dsize=(width, height))
                        order1_pic = rotated_temp[50:230, :]
                        order2_pic = rotated_temp[230:, :]

                        cv2.imwrite('pic/order.jpg', Order_temp)
                        cv2.imwrite('pic/order1.jpg', order1_pic)
                        cv2.imwrite('pic/order2.jpg', order2_pic)
                        
                        order1 = get_order_zcq(order1_pic)
                        order2 = get_order_zcq(order2_pic)
                        
                        print(order1)
                        print(order2)
                        ###################################
                        # 对准暂存区
                        arm_aim_bullseye()
                        ToAngle_adjust(third_z)
                        if list(order1.keys())[1] == 'g':
                            adjust_jjg_1(first_level_x_g, first_level_y_g, 'g')
                        elif list(order1.keys())[1] == 'b':
                            adjust_jjg_1(first_level_x_g, first_level_y_g, 'b')
                        elif list(order1.keys())[1] == 'r':
                            adjust_jjg_1(first_level_x_g, first_level_y_g, 'r')
                        ToAngle_adjust(third_z)
                        if list(order1.keys())[1] == 'g':
                            adjust_jjg_1(first_level_x_g, first_level_y_g, 'g')
                        elif list(order1.keys())[1] == 'b':
                            adjust_jjg_1(first_level_x_g, first_level_y_g, 'b')
                        elif list(order1.keys())[1] == 'r':
                            adjust_jjg_1(first_level_x_g, first_level_y_g, 'r')
                        CloseLight()
                        ###################################
                        # 抓暂存区物料
                        grab_zcq_first_level()
                        ###################################
                        # 抓完物料去精加工区
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.05*f)
                        MoveTime('back_zp', 1.2*f)
                        ToAngle_Plus(second_z)
                        if abs(get_angle(2)-second_z) > 0.5:
                            ToAngle_adjust(second_z)
                        global_value.set_value('model', 0)
                        start_z = second_z
                        MoveTime('b', 0.9*f)
                        ###################################
                        # 对准精加工
                        OpenLight()
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        ToAngle_adjust(second_z)
                        adjust_cjg_1(jjg_x, jjg_y)
                        ToAngle_adjust(second_z)
                        adjust_cjg_1(jjg_x, jjg_y)
                        CloseLight()
                        ###################################
                        # 打靶精加工
                        arm_jjg_1()
                        ###################################
                        # 打完靶后去转盘
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.23*f)
                        MoveTime('back_zp', 1.1*f)
                        ToAngle_Plus(first_z)
                        if abs(get_angle(2)-first_z) > 0.5:
                            ToAngle_adjust(first_z)
                        global_value.set_value('model', 0)
                        start_z = first_z
                        MoveTime('b', 0.41*f)
                        OpenLight()
                        time.sleep(0.5)
                        ###################################
                        # 定位转盘
                        global_value.set_value('model',1)
                        grab_see()
                        time.sleep(0.5)
                        arm_aim_turntable()
                        ToAngle_adjust(first_z)
                        adjust_zp_final(zp_final_x, zp_final_y) #定位
                        ToAngle_adjust(first_z)
                        adjust_zp_final(zp_final_x, zp_final_y) #定位
                        OpenDownLight()
                        ###################################
                        # 打靶成品区
                        arm_cpq_1()
                        ###################################
                        # 返回暂存区
                        CloseLight()
                        global_value.set_value('model', 0)
                        MoveTime('f', 0.43*f)
                        MoveTime('cm', 0.7*f)
                        ToAngle_Plus(second_z)
                        if abs(get_angle(2)-second_z) > 0.5:
                            ToAngle_adjust(second_z)
                        global_value.set_value('model', 0)
                        start_z = second_z
                        MoveTime('f', 2.30*f)
                        MoveTime('cm', 1.0*f)
                        ToAngle_Plus(third_z)
                        if abs(get_angle(2)-third_z) > 0.5:
                            ToAngle_adjust(third_z)
                        global_value.set_value('model', 0)
                        start_z = third_z
                        MoveTime('f', 1.2*f)
                        ###################################
                        # 定位暂存区
                        OpenLight()
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_see_wl()
                        ToAngle_adjust(third_z)
                        if list(order2.keys())[1] == 'g':
                            adjust_jjg_2(second_level_x_g, second_level_y_g, list(order2.keys())[1])
                        elif list(order2.keys())[1] == 'b':
                            adjust_jjg_2(second_level_x_b, second_level_y_b, list(order2.keys())[1])
                        elif list(order2.keys())[1] == 'r':
                            adjust_jjg_2(second_level_x_r, second_level_y_r, list(order2.keys())[1])
                        ToAngle_adjust(third_z)
                        if list(order2.keys())[1] == 'g':
                            adjust_jjg_2(second_level_x_g, second_level_y_g, list(order2.keys())[1])
                        elif list(order2.keys())[1] == 'b':
                            adjust_jjg_2(second_level_x_b, second_level_y_b, list(order2.keys())[1])
                        elif list(order2.keys())[1] == 'r':
                            adjust_jjg_2(second_level_x_r, second_level_y_r, list(order2.keys())[1])
                        CloseLight()
                        ###################################
                        # 取二层物料
                        grab_zcq_second_level()
                        ###################################
                        # 抓完物料去精加工区
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.05*f)
                        MoveTime('back_zp', 1.2*f)
                        ToAngle_Plus(second_z)
                        if abs(get_angle(2)-second_z) > 0.5:
                            ToAngle_adjust(second_z)
                        global_value.set_value('model', 0)
                        start_z = second_z
                        MoveTime('b', 0.9*f)
                        ###################################
                        # 对准精加工
                        OpenLight()
                        time.sleep(0.5)
                        global_value.set_value('model',1)
                        arm_aim_bullseye()
                        ToAngle_adjust(second_z)
                        adjust_cjg_1(jjg_x, jjg_y)
                        ToAngle_adjust(second_z)
                        adjust_cjg_1(jjg_x, jjg_y)
                        CloseLight()
                        ###################################
                        # 打靶精加工
                        arm_jjg_2()
                        ###################################
                        # 打完靶后去转盘
                        global_value.set_value('model', 0)
                        MoveTime('b', 1.23*f)
                        MoveTime('back_zp', 1.1*f)
                        ToAngle_Plus(first_z)
                        if abs(get_angle(2)-first_z) > 0.5:
                            ToAngle_adjust(first_z)
                        global_value.set_value('model', 0)
                        start_z = first_z
                        MoveTime('b', 0.41*f)
                        OpenLight()
                        time.sleep(0.5)
                        ###################################
                        # 定位转盘
                        global_value.set_value('model',1)
                        grab_see()
                        time.sleep(0.5)
                        arm_aim_turntable()
                        ToAngle_adjust(first_z)
                        adjust_zp_final(zp_final_x, zp_final_y) #定位
                        ToAngle_adjust(first_z)
                        adjust_zp_final(zp_final_x, zp_final_y) #定位
                        ###################################
                        # 打靶成品区
                        arm_cpq_2()
                        ###################################
                        # 回家
                        CloseLight()
                        global_value.set_value('model',0)
                        MoveTime('b', 2.25*f)
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
                arm_losses()
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
                arm_see_wl()
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
            elif mes == b'light':
                OpenLight()
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
                arm_losses()
            elif a == 4:
                arm_aim_turntable()
            elif a == 5:
                temp = global_value.get_value('frame_up')
                cv2.imwrite('pic/pic_sample/zp_final.jpg', temp)
            elif a == 7:
                arm_zp1()
            elif a == 8:
                arm_zp2()
            elif a == 9:
                arm_zp3()
            elif a == 17:
                S.write(data7)
            elif a == 18:
                S.write(data8)
            elif a == 19:
                S.write(data9)
            elif a == 16:  #back
                CloseLight()
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
                pm.write(bytearray(mes.encode()))
                pm.write(end)
            elif mes == b'zcq': #zcq
                flag = 2
                temp = global_value.get_value('frame_up')
                temp_x, temp_y = getPos_2()
                pos = '"'+str(temp_x)+','+str(temp_y)+'"'
                mes = 't0.txt='+pos
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
            elif mes == b'light':
                OpenLight()
            elif mes == b'stop':
                arm_stop()
            elif mes == b'continue':
                arm_recover()
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2:
                cv2.imwrite('pic/pic_sample/jjg.jpg', global_value.get_value('frame_up'))
            elif a == 3:
                arm_losses()
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
                CloseLight()
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
            elif mes == b'light':
                OpenLight()
            elif mes == b'stop':
                arm_stop()
            elif mes == b'continue':
                arm_recover()
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2:
                arm_interim()
            elif a == 3:
                arm_losses()
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
                CloseLight()
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
            elif mes == b'light':
                OpenLight()
            if a == 0:
                arm_initialize()
            elif a == 1:
                arm_grab()
            elif a == 2: # 采集看物料顺序
                cv2.imwrite('pic/pic_sample/order.jpg', global_value.get_value('frame_up'))
            elif a == 3:
                arm_losses()
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
                CloseLight()
                print('exit final_sample')
                break
