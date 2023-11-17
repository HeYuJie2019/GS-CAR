import RPi.GPIO as GPIO
import threading
import time
import pigpio
import numpy as np
import cv2
import serial
import struct
import serial  
import serial
import global_value
global_value._init()  #全局变量初始化，初始化字典
key = 0    #陀螺仪函数的全局变量
buff = {}  #陀螺仪函数的全局变量
wt_imu = serial.Serial("/dev/ttyAMA1", baudrate=115200) #设置陀螺仪串口
MCU = serial.Serial("/dev/ttyAMA4", baudrate=230400) #设置高精度陀螺仪串口
S = serial.Serial("/dev/ttyAMA0", 9600, bytesize=8, stopbits=1, timeout=0.5) #设置机械臂串口
pm = serial.Serial("/dev/ttyAMA3", 9600, timeout=0.1) #设置屏幕串口
qrCodeDetector = cv2.QRCodeDetector() #设置扫码实例
pi = pigpio.pi()

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
S1 = 22
S2 = 26
start_z = 0  #初始化z轴方向
y = {}
data = []  #存放扫码数据
ys = [] #存放扫码数据
t = 1.03 #摩擦系数
goForward_flag = 0 #0停止，1前进
order1 = [] #决赛半成品区顺序
order2 = []
#动作组
end = bytes.fromhex('ff ff ff') #串口屏结束符
data0 = b'\xff\x09\x00\x00\x00'
data1 = b'\xff\x09\x00\x01\x00'
data2 = b'\xff\x09\x00\x02\x00'
data3 = b'\xff\x09\x00\x03\x00'
data4 = b'\xff\x09\x00\x04\x00'
data5 = b'\xff\x09\x00\x05\x00'
data6 = b'\xff\x09\x00\x06\x00'
data7 = b'\xff\x09\x00\x07\x00'
data8 = b'\xff\x09\x00\x08\x00'
data9 = b'\xff\x09\x00\x09\x00'
data10 = b'\xff\x09\x00\x0a\x00'
data11 = bytes.fromhex('FF 09 00 0b 00')
data12 = bytes.fromhex('FF 09 00 0c 00')
data13 = bytes.fromhex('FF 09 00 0d 00')
data14 = bytes.fromhex('FF 09 00 0e 00')
data15 = bytes.fromhex('FF 09 00 0f 00')
#扫码
SAOMA1 = bytes.fromhex('FF 02 00 1d 08')
SAOMA2 = bytes.fromhex('FF 02 01 61 06')
SAOMA3 = bytes.fromhex('FF 02 02 d0 06')
SAOMA4 = bytes.fromhex('FF 02 03 ae 08')
SAOMA5 = bytes.fromhex('FF 02 04 41 03')
SAOMA6 = bytes.fromhex('FF 02 05 f4 01')
#2笔直
BIZHI1 = bytes.fromhex('FF 02 01 f2 05')
BIZHI2 = bytes.fromhex('FF 02 02 ba 05')
BIZHI3 = bytes.fromhex('FF 02 03 c6 04')
BIZHI4 = bytes.fromhex('FF 02 04 3f 08')
#初始
CHUSHI1 = bytes.fromhex('FF 02 00 1d 08')
CHUSHI2 = bytes.fromhex('FF 02 01 40 04')
CHUSHI3 = bytes.fromhex('FF 02 02 fe 03')
CHUSHI4 = bytes.fromhex('FF 02 03 c6 03')
CHUSHI5 = bytes.fromhex('FF 02 04 3f 08')
CHUSHI6 = bytes.fromhex('FF 02 05 f4 01')
#放物料-再放物料-过渡
GUODU0 = bytes.fromhex('FF 01 01 18 00')
GUODU1 = bytes.fromhex('FF 02 01 dc 05')
GUODU2 = bytes.fromhex('FF 02 02 d2 02')
GUODU3 = bytes.fromhex('FF 02 03 07 09')
GUODU4 = bytes.fromhex('FF 02 04 41 03')
#DINGWEI
DINGWEI1 = bytes.fromhex('FF 02 00 1d 08')
DINGWEI2 = bytes.fromhex('FF 02 01 61 06')
DINGWEI3 = bytes.fromhex('FF 02 02 e6 06')
DINGWEI4 = bytes.fromhex('FF 02 03 e5 08')
DINGWEI5 = bytes.fromhex('FF 02 04 3f 08')
#M0SPEED
M0_SPEED = bytes.fromhex('FF 01 00 28 00')
#JUESAI_DINGWEI 定位抓第二层物料
JUESAI_DINGWEI1 = bytes.fromhex('FF 02 00 c5 05')
JUESAI_DINGWEI2 = bytes.fromhex('FF 02 01 a4 05')
JUESAI_DINGWEI3 = bytes.fromhex('FF 02 02 c4 04')
JUESAI_DINGWEI4 = bytes.fromhex('FF 02 03 fe 02')
JUESAI_DINGWEI5 = bytes.fromhex('FF 02 04 41 03')
#粗定位看顺序
CU_DINGWEI1 = bytes.fromhex('FF 02 00 c5 05')
CU_DINGWEI2 = bytes.fromhex('FF 02 01 6c 05')
CU_DINGWEI3 = bytes.fromhex('FF 02 02 a4 04')
CU_DINGWEI4 = bytes.fromhex('FF 02 03 fe 02')
CU_DINGWEI5 = bytes.fromhex('FF 02 04 41 03')
#陀螺仪置零
zero = bytes.fromhex('FF AA 76 00 00')
#读取模板图片
standard = cv2.imread('pic/standard.jpg', cv2.IMREAD_GRAYSCALE)
standard_wl = cv2.imread('pic/standard_wl.jpg', cv2.IMREAD_GRAYSCALE)
standard_cjg = cv2.imread('pic/standard_cjg.jpg', cv2.IMREAD_GRAYSCALE)
standard_jjg = cv2.imread('pic/standard_jjg.jpg', cv2.IMREAD_GRAYSCALE)
standard_hj = cv2.imread('pic/standard_hj.jpg', cv2.IMREAD_GRAYSCALE)
standard_ylq = cv2.imread('pic/standard_ylq.jpg', cv2.IMREAD_GRAYSCALE)
standard_bcp = cv2.imread('pic/standard_bcp.jpg', cv2.IMREAD_GRAYSCALE)
################################################################
def BIZHI():
    S.write(BIZHI1)
    S.write(BIZHI2)
    S.write(BIZHI3)
    S.write(BIZHI4)
################################################################
def SAOMA():
    S.write(SAOMA1)
    S.write(SAOMA2)
    S.write(SAOMA3)
    S.write(SAOMA4)
    S.write(SAOMA5)
    S.write(SAOMA6)
################################################################
def CHUSHI():
    S.write(CHUSHI1)
    S.write(CHUSHI2)
    S.write(CHUSHI3)
    S.write(CHUSHI4)
    S.write(CHUSHI5)
    S.write(CHUSHI6)
################################################################
def GUODU():
    S.write(GUODU0)
    S.write(GUODU1)
    S.write(GUODU2)
    S.write(GUODU3)
    S.write(GUODU4)
################################################################
def DINGWEI():
    S.write(DINGWEI1)
    S.write(DINGWEI2)
    S.write(DINGWEI3)
    S.write(DINGWEI4)
    S.write(DINGWEI5)
################################################################
def JUESAI_DINGWEI():
    S.write(JUESAI_DINGWEI1)
    S.write(JUESAI_DINGWEI2)
    S.write(JUESAI_DINGWEI3)
    S.write(JUESAI_DINGWEI4)
    S.write(JUESAI_DINGWEI5)
################################################################
def CU_DINGWEI():
    S.write(CU_DINGWEI1)
    S.write(CU_DINGWEI2)
    S.write(CU_DINGWEI3)
    S.write(CU_DINGWEI4)
    S.write(CU_DINGWEI5)
################################################################
def see_zp():
    BIZHI()
    time.sleep(1)
    S.write(data0)
################################################################
def pin_init():
    GPIO.setmode(GPIO.BCM)              #select model
    GPIO_out_list = (PWMA,AIN1,AIN2,BIN1,BIN2,PWMB,PWMC,CIN1,CIN2,DIN1,DIN2,PWMD,S1,S2) #select pin
    # GPIO_in_list = (E1A, JGZ, JGY)
    # GPIO.setup(GPIO_in_list, GPIO.IN)
    GPIO.setup(GPIO_out_list, GPIO.OUT) #set pin's model
    pwm_init(PWMA, 9000, 40000)
    pwm_init(PWMB, 9000, 40000)
    pwm_init(PWMC, 9000, 40000)
    pwm_init(PWMD, 9000, 40000)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
################################################################
#初始化引脚
def pwm_init(pin, frequency, totol):
    pi.set_PWM_frequency(pin, frequency)#设定pin号引脚产生的pwm波形的频率为frequency
    pi.set_PWM_range(pin, totol) #指定要把14号引脚上的一个pwm周期分成多少份，这里是分成2000份，这个数据的范围是25-40000
    return totol
################################################################
#speed(PWM*, speed)
def speed(pin, n):
    totol = pwm_init(pin, 5000, 40000)
    num = 40000*n*0.01     #占空比 0~100
    pi.set_PWM_dutycycle(pin, num)
    return 0
################################################################
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
    elif direction == 'zuo':
        GPIO.output(AIN1,1)
        GPIO.output(AIN2,0)
        GPIO.output(BIN1,0)
        GPIO.output(BIN2,1)
        GPIO.output(CIN1,0)
        GPIO.output(CIN2,1)
        GPIO.output(DIN1,1)
        GPIO.output(DIN2,0)
    elif direction == 'you':
        GPIO.output(AIN1,0)
        GPIO.output(AIN2,1)
        GPIO.output(BIN1,1)
        GPIO.output(BIN2,0)
        GPIO.output(CIN1,1)
        GPIO.output(CIN2,0)
        GPIO.output(DIN1,0)
        GPIO.output(DIN2,1)
    elif direction == 'stop':
        speed(PWMA,0)
        speed(PWMB,0)
        speed(PWMC,0)
        speed(PWMD,0)
    else:
        speed(PWMA,0)
        speed(PWMB,0)
        speed(PWMC,0)
        speed(PWMD,0)
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
#######################################################
#颜色识别
def ColorRecognition(color, img):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == 'b':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 100, 124) #蓝色100, 124   #红色 晚上0, 10 & 白天156, 180    #绿色 晚上30, 77 白天60, 80
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, standard_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
        cv2.imwrite('sample_blue.jpg', img)
    elif color == 'r':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 0, 10) #蓝色100, 124   #红色 晚上0, 10 & 白天0-10&156-180    #绿色 晚上30, 77 白天60, 80
        h2_mask = cv2.inRange(h, 156, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask | h2_mask
        result = cv2.matchTemplate(mask, standard_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
        cv2.imwrite('sample_red.jpg', img)
    elif color == 'g':
        h, s, v = cv2.split(img_hsv)
        h_mask = cv2.inRange(h, 30, 80) #蓝色100, 124   #红色 晚上0, 10 & 白天0-10&156-180    #绿色 晚上30, 77 白天60, 85
        # h2_mask = cv2.inRange(h, 156, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
        result = cv2.matchTemplate(mask, standard_wl, cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        (startX, startY) = maxLoc
        cv2.imwrite('sample_green.jpg', img)
    else:
        (startX, startY) = (0, 0)
    return (startX, startY)
#######################################################
#陀螺仪程序
# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data
# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))
# 处理串口数据
def handleSerialData(raw_data):
    global buff
    global key
    angle_flag=False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value
        if buff[1] == 0x51 :
            if checkSum(data_buff[0:10], data_buff[10]):
                global_value.set_value('jsd', [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)])
                # acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]

        elif buff[1] == 0x53:
            if checkSum(data_buff[0:10], data_buff[10]):
                global_value.set_value('jd', [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)])
                # angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True

        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
#陀螺仪进程  JY61P
def __tly__():
    while True:
        buff_count = wt_imu.inWaiting()
        buff_data = wt_imu.read(buff_count)
        for i in range(0, buff_count):
            handleSerialData(buff_data[i])
#######################################################
#新陀螺仪程序
# 校验
def new_checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data
# 16 进制转 ieee 浮点数
def new_hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))
# 处理串口数据
def new_handleSerialData(raw_data):
    global buff
    global key
    angle_flag=False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value
        if buff[1] == 0x53:
            if new_checkSum(data_buff[0:10], data_buff[10]):
                global_value.set_value('JD', [new_hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)])
                # angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True

        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
#新陀螺仪进程  HWT101CT-TTL
def __TLY__():
    while True:
        buff_count = MCU.inWaiting()
        buff_data = MCU.read(buff_count)
        for i in range(0, buff_count):
            new_handleSerialData(buff_data[i])
#######################################################
#返回角度
def get_jd(axis,defValue=None):  
    try :
        return global_value.get_value('jd')[axis]+180
    except TypeError:
        return defValue
#返回加速度
def get_jsd(axis,defValue=None): 
    try :
        return global_value.get_value('jsd')[axis]
    except TypeError:
        return defValue
#返回新版角度
def get_new_jd(axis,defValue=None):  
    try :
        return global_value.get_value('JD')[axis]+180
    except TypeError:
        return defValue
#######################################################    
#旋转
def xuanzhuan(jiaodu):
    global goForward_flag
    goForward_flag = 0
    target_z = start_z + jiaodu
    if target_z > 360:
        flag = 1
        target_z = target_z - 360
    elif target_z < 0:
        flag = 2
        target_z = 360 + target_z
    else:
        flag = 0
    speed_xunazhuan = 0
    # print(target_z)
    while 1:
        if flag == 1:
            if get_jd(2) > 270:
                goForward_flag = -1  #旋转左
            elif get_jd(2) <= target_z - 0.1:
                goForward_flag = -1  #旋转左
            elif get_jd(2) >= target_z + 0.1:
                goForward_flag = -2  #旋转右
            elif target_z-0.1 < get_jd(2) < target_z+0.1:
                goForward_flag = 0
                break
        elif flag == 2:
            if get_jd(2) < 90:
                goForward_flag = -2  #旋转右
            elif get_jd(2) <= target_z - 0.1:
                goForward_flag = -1  #旋转左
            elif get_jd(2) >= target_z + 0.1:
                goForward_flag = -2  #旋转右
            elif target_z-0.1 < get_jd(2) < target_z+0.1:
                goForward_flag = 0
                break
        elif flag == 0:
            if get_jd(2) <= target_z - 0.1:
                goForward_flag = -1  #旋转左
            elif get_jd(2) >= target_z + 0.1:
                goForward_flag = -2  #旋转右
            elif target_z-0.1 < get_jd(2) < target_z+0.1:
                goForward_flag = 0
                break
        speed(PWMA, speed_xunazhuan)
        speed(PWMB, speed_xunazhuan)
        speed(PWMC, speed_xunazhuan)
        speed(PWMD, speed_xunazhuan)
        if abs(target_z - get_jd(2)) > 45 :
            speed_xunazhuan = 35
        elif abs(target_z - get_jd(2)) > 35 :
            speed_xunazhuan = 30
        elif abs(target_z - get_jd(2)) > 25 :
            speed_xunazhuan = 25
        elif abs(target_z - get_jd(2)) > 15 :
            speed_xunazhuan = 20
        elif abs(target_z - get_jd(2)) > 10 :
            speed_xunazhuan = 15
        elif abs(target_z - get_jd(2)) > 7 :
            speed_xunazhuan = 11
        elif abs(target_z - get_jd(2)) > 3 :
            speed_xunazhuan = 10
    goForward_flag = 0
def new_xuanzhuan(jiaodu):
    global goForward_flag
    goForward_flag = 0
    target_z = start_z + jiaodu
    if target_z > 360:
        flag = 1
        target_z = target_z - 360
    elif target_z < 0:
        flag = 2
        target_z = 360 + target_z
    else:
        flag = 0
    speed_xunazhuan = 0
    while 1:
        if flag == 1:
            if get_new_jd(2) > 270:
                goForward_flag = -1  #旋转左
            elif get_new_jd(2) <= target_z - 0.3:
                goForward_flag = -1  #旋转左
            elif get_new_jd(2) >= target_z + 0.3:
                goForward_flag = -2  #旋转右
            elif target_z-0.3 < get_new_jd(2) < target_z+0.3:
                goForward_flag = 0
                break
        elif flag == 2:
            if get_new_jd(2) < 90:
                goForward_flag = -2  #旋转右
            elif get_new_jd(2) <= target_z - 0.3:
                goForward_flag = -1  #旋转左
            elif get_new_jd(2) >= target_z + 0.3:
                goForward_flag = -2  #旋转右
            elif target_z-0.3 < get_new_jd(2) < target_z+0.3:
                goForward_flag = 0
                break
        elif flag == 0:
            if get_new_jd(2) <= target_z - 0.3:
                goForward_flag = -1  #旋转左
            elif get_new_jd(2) >= target_z + 0.3:
                goForward_flag = -2  #旋转右
            elif target_z-0.3 < get_new_jd(2) < target_z+0.3:
                goForward_flag = 0
                break
        speed(PWMA, speed_xunazhuan)
        speed(PWMB, speed_xunazhuan)
        speed(PWMC, speed_xunazhuan)
        speed(PWMD, speed_xunazhuan)
        if abs(target_z - get_new_jd(2)) > 45 :
            speed_xunazhuan = 35
        elif abs(target_z - get_new_jd(2)) > 35 :
            speed_xunazhuan = 30
        elif abs(target_z - get_new_jd(2)) > 25 :
            speed_xunazhuan = 25
        elif abs(target_z - get_new_jd(2)) > 15 :
            speed_xunazhuan = 20
        elif abs(target_z - get_new_jd(2)) > 10 :
            speed_xunazhuan = 15
        elif abs(target_z - get_new_jd(2)) > 7 :
            speed_xunazhuan = 11
        elif abs(target_z - get_new_jd(2)) > 3 :
            speed_xunazhuan = 10
    goForward_flag = 0
def ToAngle(angle):  #pid
    global goForward_flag
    speed_xunazhuan = 0
    out = 0
    Kp = 45  #45
    Ki = 15  #15
    Kd = 10  #10
    error1 = 0   #上次误差
    integral = 0	#积分和
    if 0 < angle < 90 and 360 > get_jd(2) > 270:
        while 1:
            if angle - 0.05 < get_jd(2) < angle + 0.05:
                goForward_flag = 0
                break
            elif 270 < get_jd(2) < 360:
                goForward_flag = -1  #逆时针转圈
            elif get_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 30):
                speed_xunazhuan = 30
            # print('out: ', out, '  speed: ', speed_xunazhuan)
    elif 360 > angle > 270 and 0 < get_jd(2) < 90:
        while 1:
            if angle - 0.2 < get_jd(2) < angle + 0.2:
                goForward_flag = 0
                break
            elif 0 < get_jd(2) < 90:
                goForward_flag = -2  #顺时针转圈
            elif get_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 30):
                speed_xunazhuan = 30
            # print('out: ', out, '  speed: ', speed_xunazhuan)
    else:
        while 1:
            if angle - 0.2 < get_jd(2) < angle + 0.2:
                goForward_flag = 0
                break
            elif get_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 30):
                speed_xunazhuan = 30
            # print('out: ', out, '  speed: ', speed_xunazhuan)
def new_ToAngle(angle):  #pid
    global goForward_flag
    speed_xunazhuan = 0
    out = 0
    Kp = 10  #10
    Ki = 3  #3
    Kd = 2  #2
    error1 = 0   #上次误差
    integral = 0	#积分和
    if 0 < angle < 90 and 360 > get_new_jd(2) > 270:
        t1 = t2 = time.time()
        while 1:
            t2 = time.time()
            if t2 - t1 > 5:
                goForward_flag = 0
                break
            if angle - 0.05 < get_new_jd(2) < angle + 0.05:
                goForward_flag = 0
                break
            elif 270 < get_new_jd(2) < 360:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_new_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 30):
                speed_xunazhuan = 30
            # print('out: ', out, '  speed: ', speed_xunazhuan)
    elif 360 > angle > 270 and 0 < get_new_jd(2) < 90:
        t1 = t2 = time.time()
        while 1:
            t2 = time.time()
            if t2 - t1 > 5:
                goForward_flag = 0
                break
            if angle - 0.05 < get_new_jd(2) < angle + 0.05:
                goForward_flag = 0
                break
            elif 0 < get_new_jd(2) < 90:
                goForward_flag = -2  #顺时针转圈
            elif get_new_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_new_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 30):
                speed_xunazhuan = 30
            # print('out: ', out, '  speed: ', speed_xunazhuan)
    else:
        t1 = t2 = time.time()
        while 1:
            t2 = time.time()
            if t2 - t1 > 5:
                goForward_flag = 0
                break
            if angle - 0.05 < get_new_jd(2) < angle + 0.05:
                goForward_flag = 0
                break
            elif get_new_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_new_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 30):
                speed_xunazhuan = 30
            # print('out: ', out, '  speed: ', speed_xunazhuan)
def approximate_ToAngle(angle):  #pid
    global goForward_flag
    speed_xunazhuan = 0
    out = 0
    Kp = 3  #3
    Ki = 6  #5
    Kd = 30  #13
    error1 = 0   #上次误差
    integral = 0	#积分和
    if 0 < angle < 90 and 360 > get_new_jd(2) > 270:
        t1 = t2 = time.time()
        while 1:
            t2 = time.time()
            if t2 - t1 > 4:
                goForward_flag = 0
                break
            if angle - 0.5 < get_new_jd(2) < angle + 0.5:
                goForward_flag = 0
                break
            elif 270 < get_new_jd(2) < 360:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_new_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 20):
                speed_xunazhuan = 20
            # print('out: ', out, '  speed: ', speed_xunazhuan)
    elif 360 > angle > 270 and 0 < get_new_jd(2) < 90:
        t1 = t2 = time.time()
        while 1:
            t2 = time.time()
            if t2 - t1 > 4:
                goForward_flag = 0
                break
            if angle - 0.5 < get_new_jd(2) < angle + 0.5:
                goForward_flag = 0
                break
            elif 0 < get_new_jd(2) < 90:
                goForward_flag = -2  #顺时针转圈
            elif get_new_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_new_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 20):
                speed_xunazhuan = 20
            # print('out: ', out, '  speed: ', speed_xunazhuan)
    else:
        t1 = t2 = time.time()
        while 1:
            t2 = time.time()
            if t2 - t1 > 4:
                goForward_flag = 0
                break
            if angle - 0.5 < get_new_jd(2) < angle + 0.5:
                goForward_flag = 0
                break
            elif get_new_jd(2) < angle-0.2:
                goForward_flag = -1  #逆时针转圈
            elif get_new_jd(2) > angle+0.2:
                goForward_flag = -2  #顺时针转圈
            speed(PWMA, speed_xunazhuan)
            speed(PWMB, speed_xunazhuan)
            speed(PWMC, speed_xunazhuan)
            speed(PWMD, speed_xunazhuan)
            error0 = angle - get_new_jd(2)
            integral = integral + error0
            if integral > 100:
                integral = 100
            elif integral < -100:
                integral = -100
            derivative = error0 - error1
            out = Kp*error0 + Ki*integral + Kd*derivative
            speed_xunazhuan = abs(out/((Kp + Ki + Kd)*100))*100
            if(speed_xunazhuan > 20):
                speed_xunazhuan = 20
            # print('out: ', out, '  speed: ', speed_xunazhuan)
#######################################################
#显示数字
def display_num(index, num):
    mes = 'n'+str(index)+'.val='+str(num)   #n0.val=0
    pm.write(bytearray(mes.encode()))
    pm.write(end)
#读取屏幕指令
def Read_pm():
    try:
        a = struct.unpack('<hh', pm.read_all())
    except:
        a = (-1, 0)
    return a[0]
#刷新屏幕界面
def Page_pm(i):
    mes = 'page page' + str(i)
    pm.write(bytearray(mes.encode()))
    pm.write(end)
#######################################################
#延时行走
def move_time(n):
    global goForward_flag
    t1 = time.time()
    t2 = time.time()
    goForward_flag = 1
    while t2-t1 < n*t:
        t2 = time.time()
    goForward_flag = 0
    # print(t2-t1)
def move_time_2(n):
    global goForward_flag
    t1 = time.time()
    t2 = time.time()
    goForward_flag = 2
    while t2-t1 < n*t:
        t2 = time.time()
    goForward_flag = 0
#######################################################
#行走线程
def goForward():
    global goForward_flag
    global start_z
    while 1:
        if goForward_flag == 1:
            move('front')
            if get_new_jd(2) - start_z < -2: #向右偏
                speed(PWMA,25)  #20
                speed(PWMB,20)  #20
                speed(PWMC,40)  #40
                speed(PWMD,42)  #37
            elif get_new_jd(2) - start_z < -1: #向右偏
                speed(PWMA,22)  #20
                speed(PWMB,20)  #20
                speed(PWMC,40)  #40
                speed(PWMD,39)  #37
            elif get_new_jd(2) - start_z > 2: #向左偏
                speed(PWMA,20)  #20
                speed(PWMB,25)  #20
                speed(PWMC,45)  #40
                speed(PWMD,37)  #37
            elif get_new_jd(2) - start_z > 1: #向左偏
                speed(PWMA,20)  #20
                speed(PWMB,22)  #20
                speed(PWMC,42)  #40
                speed(PWMD,37)  #37
            else:
                speed(PWMA,20)  #20
                speed(PWMB,20)  #20
                speed(PWMC,40)  #40
                speed(PWMD,37)  #37
        elif goForward_flag == 1.5:
            move('front')
            if get_new_jd(2) - start_z < -1: #向右偏得更多
                speed(PWMA,19)  #15
                speed(PWMB,15)  #15
                speed(PWMC,15)  #15
                speed(PWMD,19)  #15
            elif get_new_jd(2) - start_z < -0.3: #向右偏
                speed(PWMA,17)  #15
                speed(PWMB,15)  #15
                speed(PWMC,15)  #15
                speed(PWMD,17)  #15
            elif get_new_jd(2) - start_z > 1: #向左偏得更多
                speed(PWMA,15)  #15
                speed(PWMB,19)  #15
                speed(PWMC,19)  #15
                speed(PWMD,15)  #15
            elif get_new_jd(2) - start_z > 0.3: #向左偏
                speed(PWMA,15)  #15
                speed(PWMB,17)  #15
                speed(PWMC,17)  #15
                speed(PWMD,15)  #15
            else:
                speed(PWMA,15)  #15
                speed(PWMB,15)  #15
                speed(PWMC,15)  #15
                speed(PWMD,15)  #15
        elif goForward_flag == 0:
            move('stop')
        elif goForward_flag == 2:
            move('back')
            if get_new_jd(2) - start_z < -2: #向右偏
                speed(PWMA,40)  #20
                speed(PWMB,45)  #20
                speed(PWMC,25)  #40
                speed(PWMD,20)  #37
            elif get_new_jd(2) - start_z < -1: #向右偏
                speed(PWMA,40)  #20
                speed(PWMB,45)  #20
                speed(PWMC,22)  #40
                speed(PWMD,20)  #37
            elif get_new_jd(2) - start_z > 2: #向左偏
                speed(PWMA,45)  #20
                speed(PWMB,40)  #20
                speed(PWMC,20)  #40
                speed(PWMD,25)  #37
            elif get_new_jd(2) - start_z > 1: #向左偏
                speed(PWMA,42)  #20
                speed(PWMB,40)  #20
                speed(PWMC,20)  #40
                speed(PWMD,22)  #37
            else:
                speed(PWMA,40)  #20
                speed(PWMB,40)  #20
                speed(PWMC,20)  #40
                speed(PWMD,20)  #37
        elif goForward_flag == 2.5:
            move('back')
            if get_new_jd(2) - start_z < -1: #向右偏得更多
                speed(PWMA,15)  #15
                speed(PWMB,19)  #15
                speed(PWMC,19)  #15
                speed(PWMD,15)  #15
            elif get_new_jd(2) - start_z < -0.3: #向右偏
                speed(PWMA,15)  #15
                speed(PWMB,17)  #15
                speed(PWMC,17)  #15
                speed(PWMD,15)  #15
            elif get_new_jd(2) - start_z > 1: #向左偏得更多
                speed(PWMA,19)  #15
                speed(PWMB,15)  #15
                speed(PWMC,15)  #15
                speed(PWMD,19)  #15
            elif get_new_jd(2) - start_z > 0.3: #向左偏
                speed(PWMA,17)  #15
                speed(PWMB,15)  #15
                speed(PWMC,15)  #15
                speed(PWMD,17)  #15
            else:
                speed(PWMA,15)  #15
                speed(PWMB,15)  #15
                speed(PWMC,15)  #15
                speed(PWMD,15)  #15
        #左移
        elif goForward_flag == 3:
            move('zuo')
            if get_new_jd(2) - start_z < -1: #向右偏得更多
                speed(PWMB,25)  #20
                speed(PWMC,20)  #20
                speed(PWMA,25)  #20
                speed(PWMD,20)  #20
            elif get_new_jd(2) - start_z < -0.3: #向右偏
                speed(PWMB,22)  #20
                speed(PWMC,20)  #20
                speed(PWMA,22)  #20
                speed(PWMD,20)  #20
            elif get_new_jd(2) - start_z > 1: #向左偏得更多
                speed(PWMB,20)  #20
                speed(PWMC,25)  #20
                speed(PWMA,20)  #20
                speed(PWMD,25)  #20
            elif get_new_jd(2) - start_z > 0.3: #向左偏
                speed(PWMB,20)  #20
                speed(PWMC,22)  #20
                speed(PWMA,20)  #20
                speed(PWMD,22)  #20
            else:
                speed(PWMB,20)  #20
                speed(PWMC,20)  #20
                speed(PWMA,20)  #20
                speed(PWMD,20)  #20
        #右移
        elif goForward_flag == 4:
            move('you')
            if get_new_jd(2) - start_z < -1: #向右偏得更多
                speed(PWMB,20)  #20
                speed(PWMC,25)  #20
                speed(PWMA,20)  #20
                speed(PWMD,25)  #20
            elif get_new_jd(2) - start_z < -0.3: #向右偏
                speed(PWMB,20)  #20
                speed(PWMC,22)  #20
                speed(PWMA,20)  #20
                speed(PWMD,22)  #20
            elif get_new_jd(2) - start_z > 1: #向左偏得更多
                speed(PWMB,25)  #20
                speed(PWMC,20)  #20
                speed(PWMA,25)  #20
                speed(PWMD,20)  #20
            elif get_new_jd(2) - start_z > 0.3: #向左偏
                speed(PWMB,22)  #20
                speed(PWMC,20)  #20
                speed(PWMA,22)  #20
                speed(PWMD,20)  #20
            else:
                speed(PWMB,20)  #20
                speed(PWMC,20)  #20
                speed(PWMA,20)  #20
                speed(PWMD,20)  #20
        elif goForward_flag == -1:
            move('xuanzhuanzuo')
        elif goForward_flag == -2:
            move('xuanzhuanyou')
        #出门
        elif goForward_flag == 6:
            move('front')
            speed(PWMA,35)  #33
            speed(PWMB,0)  #30
            speed(PWMC,35)  #50
            speed(PWMD,0)  #50
        #回家
        elif goForward_flag == 7:
            move('front')
            speed(PWMA,0)  #33
            speed(PWMB,36)  #30
            speed(PWMC,0)  #50
            speed(PWMD,35)  #50
#######################################################
#定位圆盘，返回坐标
def DW1(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    h_mask = cv2.inRange(h, 24, 34) 
    s_mask = cv2.inRange(s, 43, 255)
    v_mask = cv2.inRange(v, 46, 255)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, standard, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (startX, startY) = maxLoc
    return maxLoc
#定位加工区域
def DW2(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    h_mask = cv2.inRange(h, 31, 90)  #晚上31-77 白天60-(77-90)
    s_mask = cv2.inRange(s, 43, 255) #43-15
    v_mask = cv2.inRange(v, 46, 255) #46-15
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, standard_cjg, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (startX, startY) = maxLoc
    return maxLoc
#定位精加工区域
def DW3(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    h_mask = cv2.inRange(h, 40, 80)  #晚上40-80 白天60-80
    s_mask = cv2.inRange(s, 43, 255) #晚上30-255 白天25-255
    v_mask = cv2.inRange(v, 46, 255) #晚上30-255 白天25-255
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, standard_jjg, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (startX, startY) = maxLoc
    return maxLoc
#定位回家
def DW4(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    h_mask = cv2.inRange(h, 25, 30)
    s_mask = cv2.inRange(s, 43, 255)
    v_mask = cv2.inRange(v, 46, 255)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, standard_hj, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (startX, startY) = maxLoc
    return maxLoc
#原料区抓取物料
def DW5(frame, color):
    frame = frame[200:400]
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    if color == 'g':
        h_mask = cv2.inRange(h, 31, 90)  #晚上31-77 白天60-(77-90)
        s_mask = cv2.inRange(s, 43, 255) #43-15
        v_mask = cv2.inRange(v, 46, 255) #46-15
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 100, 124)  #蓝色100, 124   #红色 晚上0-10&156-180 | 白天0-3&170-180    #绿色 晚上30, 80 白天60, 80
        s_mask = cv2.inRange(s, 10, 255)
        v_mask = cv2.inRange(v, 10, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'r':
        h_mask = cv2.inRange(h, 0, 3)  #蓝色100, 124   #红色 晚上0-10&156-180 | 白天0-3&170-180    #绿色 晚上30, 80 白天60, 80
        h2_mask = cv2.inRange(h, 170, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask | h2_mask
    result = cv2.matchTemplate(mask, standard_ylq, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (startX, startY) = maxLoc
    return startX, startY, mask
#粗定位看顺序
def DW6(frame):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    h_mask = cv2.inRange(h, 24, 34) 
    s_mask = cv2.inRange(s, 43, 255)
    v_mask = cv2.inRange(v, 46, 255)
    mask = h_mask & s_mask & v_mask
    result = cv2.matchTemplate(mask, standard_bcp, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (startX, startY) = maxLoc
    return maxLoc
#决赛根据顺序抓取物料
def DW7(frame, color):
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    if color == 'g':
        h_mask = cv2.inRange(h, 31, 90)  #晚上31-77 白天60-(77-90)
        s_mask = cv2.inRange(s, 43, 255) #43-15
        v_mask = cv2.inRange(v, 46, 255) #46-15
        mask = h_mask & s_mask & v_mask
    elif color == 'b':
        h_mask = cv2.inRange(h, 100, 124)  #蓝色100, 124   #红色 晚上0-10&156-180 | 白天0-3&170-180    #绿色 晚上30, 80 白天60, 80
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask
    elif color == 'r':
        h_mask = cv2.inRange(h, 0, 3)  #蓝色100, 124   #红色 晚上0-10&156-180 | 白天0-3&170-180    #绿色 晚上30, 80 白天60, 80
        h2_mask = cv2.inRange(h, 170, 180)
        s_mask = cv2.inRange(s, 43, 255)
        v_mask = cv2.inRange(v, 46, 255)
        mask = h_mask & s_mask & v_mask | h2_mask
    result = cv2.matchTemplate(mask, standard_jjg, cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (startX, startY) = maxLoc
    return maxLoc
#######################################################
#调整函数，在圆盘旁边
def adjust(limtX, limtY):
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW1(frame)
    t1 = time.time()
    t2 = time.time()
    while (limtX - 10 < x < limtX + 10 and limtY - 10 < y < limtY + 10) == 0:
        t2 = time.time()
        if t2-t1 > 30:
            break
        while x > limtX + 10 or x < limtX - 10:
            time.sleep(1.0*t)
            if x > limtX + 20 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX + 20 >= x >= limtX + 10:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX - 20 <= x <= limtX - 10:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x < limtX - 20:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW1(frame)
        GPIO.output(S1, 1)
        GPIO.output(S2, 1)
        new_ToAngle(start_z)
        time.sleep(0.2)
        GPIO.output(S1, 0)
        GPIO.output(S2, 0)
        while y > limtY + 10 or y < limtY - 10:
            time.sleep(1.0*t)
            if y > limtY + 20 :
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY + 20 >= y >= limtY + 10:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY - 20 <= y <= limtY - 10:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y < limtY - 20:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW1(frame)
    # new_ToAngle(start_z)
    cv2.imwrite('adjust1.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 2)
def super_adjust(limtX, limtY):
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW1(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX - 5 < x < limtX + 5 and limtY - 5 < y < limtY + 5) == 0:
        t3 = time.time()
        t4 = time.time()
        while x >= limtX + 5 or x <= limtX - 5:
            t4 = time.time()
            if t4-t3 > 8:
                break
            time.sleep(1.0*t)
            if x > limtX + 15 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX + 15 >= x >= limtX + 5:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX - 15 <= x <= limtX - 5:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x < limtX - 15:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW1(frame)
        t2_totol = time.time()
        if t2_totol-t1_totol > 40:
            break
        GPIO.output(S1, 1)
        GPIO.output(S2, 1)
        new_ToAngle(start_z)
        time.sleep(0.2)
        GPIO.output(S1, 0)
        GPIO.output(S2, 0)
        # t2_totol = time.time()
        # if t2_totol-t1_totol > 30:
        #     break
        t5 = time.time()
        t6 = time.time
        while y >= limtY + 5 or y <= limtY - 5:
            t6 = time.time()
            if t6 - t5 > 8:
                break
            time.sleep(1.0*t)
            if y > limtY + 15 :
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY + 15 >= y >= limtY + 5:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY - 15 <= y <= limtY - 5:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y < limtY - 15:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW1(frame)
    # new_ToAngle(start_z)
    cv2.imwrite('adjust1.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 2)
def adjust2(limtX, limtY):  #看靶心调整
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW2(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX-10 < x <limtX+10 and limtY-10 < y < limtY+10) == 0:
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t3 = time.time()
        t4 = time.time()
        while x >= limtX+10 or x <= limtX-10:
            t4 = time.time()
            if t4-t3 > 10:
                break
            time.sleep(1.0*t)
            if x >= limtX+20 :
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.18*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX+20 > x >= limtX+10:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX-20 < x <= limtX-10:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x <= limtX-20:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.18*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW2(frame)
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t5 = time.time()
        t6 = time.time()
        while y >= limtY+10 or y <= limtY-10:
            t6 = time.time()
            if t6-t5 > 10:
                break
            time.sleep(1.0*t)
            if y >= limtY+20 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.2*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY+20 > y >= limtY+10:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY-20 < y <= limtY-10:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y <= limtY-20:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.2*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW2(frame)
    cv2.imwrite('adjust2.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
def super_adjust2(limtX, limtY):
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW2(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX-4 < x < limtX+4 and limtY-4 < y < limtY+4) == 0:
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t3 = time.time()
        t4 = time.time()
        while x >= limtX+4 or x <= limtX-4:
            t4 = time.time()
            if t4-t3 > 10:
                break
            time.sleep(1.0*t)
            if x >= limtX+10 :
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX+10 > x >= limtX+4:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.05*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX-10 < x <= limtX-4:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.05*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x <= limtX-10:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW2(frame)
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t5 = time.time()
        t6 = time.time()
        while y >= limtY+4 or y <= limtY-4:
            t6 = time.time()
            if t6 - t5 > 10:
                break
            time.sleep(1.0*t)
            if y >= limtY+10 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY+10 > y >= limtY+4:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY-10 < y <= limtY-4:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y <= limtY-10:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW2(frame)
        GPIO.output(S1, 1)
        GPIO.output(S2, 1)
        new_ToAngle(start_z)
        time.sleep(0.2)
        GPIO.output(S1, 0)
        GPIO.output(S2, 0)
        frame = global_value.get_value('frame')
        x, y = DW2(frame)
    cv2.imwrite('adjust2.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
def adjust3(limtX, limtY):
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW3(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX-10 < x < limtX+10 and limtY-10 < y < limtY+10) == 0:
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t3 = time.time()
        t4 = time.time()
        while x > limtX + 10 or x < limtX - 10:
            t4 = time.time()
            if t4-t3 > 20:
                break
            time.sleep(1*t)
            if x >= limtX + 20 :
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.12*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX + 20 > x >= limtX + 10:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.06*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX - 20 < x <= limtX - 10:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.06*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x <= limtX - 20:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.12*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW3(frame)
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t5 = time.time()
        t6 = time.time()
        while y > limtY + 10 or y < limtY - 10:
            t6 = time.time()
            if t6-t5 > 20:
                break
            time.sleep(1*t)
            if y > limtY + 20 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY + 20 > y > limtY + 10:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.08*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY - 20 < y < limtY - 10:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.08*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y < limtY - 20:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW3(frame)
    cv2.imwrite('adjust3.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
def super_adjust3(limtX, limtY):
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW3(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX-4 < x < limtX+4 and limtY-4 < y < limtY+4) == 0:
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t3 = time.time()
        t4 = time.time()
        while x >= limtX+4 or x <= limtX-4:
            t4 = time.time()
            if t4-t3 > 10:
                break
            time.sleep(1.0*t)
            if x >= limtX+10 :
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX+10 > x >= limtX+4:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.05*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX-10 < x <= limtX-4:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.05*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x <= limtX-10:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW3(frame)
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t5 = time.time()
        t6 = time.time()
        while y >= limtY+4 or y <= limtY-4:
            t6 = time.time()
            if t6-t5 > 10:
                break
            time.sleep(1.0*t)
            if y >= limtY+10 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY+10 > y >= limtY+4:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY-10 < y <= limtY-4:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y <= limtY-10:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW3(frame)
        GPIO.output(S1, 1)
        GPIO.output(S2, 1)
        new_ToAngle(start_z)
        time.sleep(0.2)
        GPIO.output(S1, 0)
        GPIO.output(S2, 0)
        frame = global_value.get_value('frame')
        x, y = DW3(frame)
    cv2.imwrite('adjust3.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
def adjust4(limtX, limtY): #定位回家
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW4(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while x > limtX + 10 or x < limtX + 10:
        t2_totol = time.time()
        if t2_totol-t1_totol > 10:
            break
        time.sleep(0.5*t)
        if x > limtX + 20 :
            goForward_flag = 2.5
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.2*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        elif limtX + 20 >= x >= limtX + 10:
            goForward_flag = 2.5
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.1*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        elif limtX - 20 <= x <= limtX - 10:
            goForward_flag = 1.5
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.1*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        elif x < limtX - 20:
            goForward_flag = 1.5
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.2*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        frame = global_value.get_value('frame')
        x, y = DW4(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while y > limtY + 10 or y < limtY - 10:
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        time.sleep(0.5*t)
        if y > limtY + 20 :
            goForward_flag = 3
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.2*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        elif limtY + 20 >= y >= limtY + 10:
            goForward_flag = 3
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.1*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        elif limtY - 20 <= y <= limtY - 10:
            goForward_flag = 4
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.1*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        elif y < limtY - 20:
            goForward_flag = 4
            time.sleep(0.1)
            t1 = time.time()
            t2 = time.time()
            while t2-t1 < 0.2*t:
                t2 = time.time()
                GPIO.output(S1, 1)
                GPIO.output(S2, 1)
            goForward_flag = 0
            time.sleep(0.2)
            GPIO.output(S1, 0)
            GPIO.output(S2, 0)
        frame = global_value.get_value('frame')
        x, y = DW4(frame)
    cv2.imwrite('adjust4.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
def adjust5(limtX, limtY): #粗定位看顺序  用DW5
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW6(frame)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX-20 < x < limtX+20 and limtY-20 < y < limtY+20) == 0:
        t2_totol = time.time()
        if t2_totol-t1_totol > 40:
            break
        t3 = time.time()
        t4 = time.time()
        while x >= limtX + 20 or x <= limtX - 20:
            t4 = time.time()
            if t4-t3 > 20:
                break
            time.sleep(1*t)
            if x > limtX + 40 :
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.12*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX + 40 >= x >= limtX + 20:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.06*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX - 40 <= x <= limtX - 20:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.06*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x < limtX - 40:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.12*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW6(frame)
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t5 = time.time()
        t6 = time.time()
        while y >= limtY + 20 or y <= limtY - 20:
            t6 = time.time()
            if t6-t5 > 20:
                break
            time.sleep(1*t)
            if y > limtY + 40 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY + 40 >= y >= limtY + 20:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.08*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY - 40 <= y <= limtY - 20:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.08*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y < limtY - 40:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW6(frame)
    cv2.imwrite('adjust5.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
def adjust6(limtX, limtY, flag): #决赛定位抓取  用DW7
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW7(frame, flag)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX-10 < x < limtX+10 and limtY-10 < y < limtY+10) == 0:
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t3 = time.time()
        t4 = time.time()
        while x > limtX + 10 or x < limtX - 10:
            t4 = time.time()
            if t4-t3 > 20:
                break
            time.sleep(1*t)
            if x >= limtX + 20 :
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.12*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX + 20 > x >= limtX + 10:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.06*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX - 20 < x <= limtX - 10:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.06*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x <= limtX - 20:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.12*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW7(frame, flag)
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t5 = time.time()
        t6 = time.time()
        while y > limtY + 10 or y < limtY - 10:
            t6 = time.time()
            if t6-t5 > 20:
                break
            time.sleep(1*t)
            if y > limtY + 20 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY + 20 > y > limtY + 10:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.08*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY - 20 < y < limtY - 10:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.08*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y < limtY - 20:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW7(frame, flag)
    cv2.imwrite('adjust6.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
def super_adjust6(limtX, limtY, flag):  #用DW7
    GPIO.output(S1, 0)
    GPIO.output(S2, 0)
    global goForward_flag
    frame = global_value.get_value('frame')
    x, y = DW7(frame, flag)
    t1_totol = time.time()
    t2_totol = time.time()
    while (limtX-4 < x < limtX+4 and limtY-4 < y < limtY+4) == 0:
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t3 = time.time()
        t4 = time.time()
        while x >= limtX+4 or x <= limtX-4:
            t4 = time.time()
            if t4-t3 > 10:
                break
            time.sleep(1.0*t)
            if x >= limtX+10 :
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX+10 > x >= limtX+4:
                goForward_flag = 4
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.05*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtX-10 < x <= limtX-4:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.05*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif x <= limtX-10:
                goForward_flag = 3
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.1*t:
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW7(frame, flag)
        t2_totol = time.time()
        if t2_totol-t1_totol > 30:
            break
        t5 = time.time()
        t6 = time.time()
        while y >= limtY+4 or y <= limtY-4:
            t6 = time.time()
            if t6-t5 > 10:
                break
            time.sleep(1.0*t)
            if y >= limtY+10 :
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY+10 > y >= limtY+4:
                goForward_flag = 2.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif limtY-10 < y <= limtY-4:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.07*t:  #0.1
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            elif y <= limtY-10:
                goForward_flag = 1.5
                time.sleep(0.1)
                t1 = time.time()
                t2 = time.time()
                while t2-t1 < 0.15*t:  #0.2
                    t2 = time.time()
                    GPIO.output(S1, 1)
                    GPIO.output(S2, 1)
                goForward_flag = 0
                time.sleep(0.2)
                GPIO.output(S1, 0)
                GPIO.output(S2, 0)
            frame = global_value.get_value('frame')
            x, y = DW7(frame, flag)
        GPIO.output(S1, 1)
        GPIO.output(S2, 1)
        new_ToAngle(start_z)
        time.sleep(0.2)
        GPIO.output(S1, 0)
        GPIO.output(S2, 0)
        frame = global_value.get_value('frame')
        x, y = DW3(frame)
    cv2.imwrite('adjust3.jpg', frame)
    GPIO.output(S1, 1)
    GPIO.output(S2, 1)
#######################################################
#视频进程
def getFrame():
    cap = cv2.VideoCapture(0)
    while 1:
        _, frame = cap.read()
        global_value.set_value('frame', frame)
#######################################################
#判断转盘是运动还是静止
def MoveOrStatic(color):
    frame1 = global_value.get_value('frame')
    x1, y1 = ColorRecognition(color, frame1)
    frame2 = global_value.get_value('frame')
    x2, y2 = ColorRecognition(color, frame2)
    # return abs(x2-x1) + abs(y2-y1)
    if abs(x2-x1) + abs(y2-y1) >= 3:
        return 0   #move
    elif abs(x2-x1) + abs(y2-y1) < 3:
        return 1   #static
def MoveOrStatic_2(color):
    frame1 = global_value.get_value('frame')
    x1, y1, _ = DW5(frame1, color)
    frame2 = global_value.get_value('frame')
    x2, y2, _ = DW5(frame2, color)
    # return abs(x2-x1) + abs(y2-y1)
    if abs(x2-x1) > 3:
        return 0   #move
    else:
        return 1   #static
#######################################################
#抓转盘物料
def Grab_zhuanpan():
    frame = global_value.get_value('frame')
    x1, y1 = ColorRecognition(ys[0], frame)
    x2, y2 = ColorRecognition(ys[1], frame)
    x3, y3 = ColorRecognition(ys[2], frame)
    if y1 > y2 and y1 > y3 and MoveOrStatic(ys[2]) == 1:
        print('special condition!')
        time.sleep(6)
    elif y1 < 128 and MoveOrStatic(ys[2]) == 1:
        print('special condition!')
        time.sleep(6)
    #1
    frame = global_value.get_value('frame')
    x, y = ColorRecognition(ys[0], frame)
    while MoveOrStatic(ys[0]) == 0 or y < 50:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y = ColorRecognition(ys[0], frame)
    if x < 172:
        S.write(data3)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 172 <= x < 350:
        S.write(data2)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 350 <= x:
        S.write(data4)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    S.write(data5)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #2
    frame = global_value.get_value('frame')
    x, y = ColorRecognition(ys[1], frame)
    while MoveOrStatic(ys[1]) == 0 or y < 50 : #ys[1]
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y = ColorRecognition(ys[1], frame)
    if x < 172:
        S.write(data3)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 172 <= x < 350:
        S.write(data2)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 350 <= x:
        S.write(data4)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    S.write(data6)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #3
    frame = global_value.get_value('frame')
    x, y = ColorRecognition(ys[2], frame)
    while MoveOrStatic(ys[2]) == 0 or y < 50 : #ys[2]
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y = ColorRecognition(ys[2], frame)
    if x < 172:
        S.write(data3)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 172 <= x < 350:
        S.write(data2)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 350 <= x:
        S.write(data4)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    S.write(data7)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(1)
    CHUSHI()
#######################################################
def Grab_zhuanpan_2():
    frame = global_value.get_value('frame')
    x1, y1 = ColorRecognition(ys[3], frame)
    x2, y2 = ColorRecognition(ys[4], frame)
    x3, y3 = ColorRecognition(ys[5], frame)
    if y1 > y2 and y1 > y3 and MoveOrStatic(ys[3]) == 1:
        print('special condition!')
        time.sleep(6)
    elif y1 < 128 and MoveOrStatic(ys[3]) == 1:
        print('special condition!')
        time.sleep(6)
    #1
    frame = global_value.get_value('frame')
    x, y = ColorRecognition(ys[3], frame)
    while MoveOrStatic(ys[3]) == 0 or y < 50:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y = ColorRecognition(ys[3], frame)
    if x < 172:
        S.write(data3)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 172 <= x < 350:
        S.write(data2)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 350 <= x:
        S.write(data4)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    S.write(data5)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #2
    frame = global_value.get_value('frame')
    x, y = ColorRecognition(ys[4], frame)
    while MoveOrStatic(ys[4]) == 0 or y < 50 : #ys[1]
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y = ColorRecognition(ys[4], frame)
    if x < 172:
        S.write(data3)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 172 <= x < 350:
        S.write(data2)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 350 <= x:
        S.write(data4)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    S.write(data6)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #3
    frame = global_value.get_value('frame')
    x, y = ColorRecognition(ys[5], frame)
    while MoveOrStatic(ys[5]) == 0 or y < 50 : #ys[2]
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y = ColorRecognition(ys[5], frame)
    if x < 172:
        S.write(data3)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 172 <= x < 350:
        S.write(data2)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    elif 350 <= x:
        S.write(data4)
        time.sleep(1.8)
        S.write(data15)
        time.sleep(0.5)
    S.write(data7)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(1)
    CHUSHI()
#######################################################
def Place_wl_4():
    BIZHI()
    time.sleep(1.2)
    S.write(M0_SPEED)
    S.write(data5)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    frame = global_value.get_value('frame')
    x1, y1, _ = DW5(frame, ys[0])
    x2, y2, _ = DW5(frame, ys[1])
    x3, y3, _ = DW5(frame, ys[2])
    if x2 < x1 < x3 and MoveOrStatic_2(ys[0]) == 1:
        print('special condition!')
        time.sleep(6)
    #1
    frame = global_value.get_value('frame')
    x, y, _ = DW5(frame, ys[0])
    while MoveOrStatic_2(ys[0]) == 0 or x < 203 or x > 311:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y, mask = DW5(frame, ys[0])
    print('mask1: ', np.sum(mask), 'x: ', x)
    cv2.imwrite('place1.jpg', global_value.get_value('frame'))
    S.write(data2)
    time.sleep(1.8)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(0.8)
    S.write(data6)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #2
    frame = global_value.get_value('frame')
    x, y, _ = DW5(frame, ys[1])
    while MoveOrStatic_2(ys[1]) == 0 or x < 203 or x > 311:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y, mask = DW5(frame, ys[1])
    print('mask2: ', np.sum(mask), 'x: ', x)
    cv2.imwrite('place2.jpg', global_value.get_value('frame'))
    S.write(data2)
    time.sleep(1.8)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(0.8)
    S.write(data7)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #3
    frame = global_value.get_value('frame')
    x, y, _ = DW5(frame, ys[2])
    while MoveOrStatic_2(ys[2]) == 0 or x < 203 or x > 311:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y, mask = DW5(frame, ys[2])
    print('mask3: ', np.sum(mask), 'x: ', x)
    cv2.imwrite('place3.jpg', global_value.get_value('frame'))
    S.write(data2)
    time.sleep(1.8)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(1)
    CHUSHI()
def Place_wl_5():
    BIZHI()
    time.sleep(1.2)
    S.write(M0_SPEED)
    S.write(data5)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    frame = global_value.get_value('frame')
    x1, y1, _ = DW5(frame, ys[3])
    x2, y2, _ = DW5(frame, ys[4])
    x3, y3, _ = DW5(frame, ys[5])
    if x2 < x1 < x3 and MoveOrStatic_2(ys[0]) == 1:
        print('special condition!')
        time.sleep(6)
    #1
    frame = global_value.get_value('frame')
    x, y, mask = DW5(frame, ys[3])
    while MoveOrStatic_2(ys[3]) == 0 or x < 203 or x > 311 or np.sum(mask) > 3000000:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y, mask = DW5(frame, ys[3])
    print('mask4: ', np.sum(mask))
    cv2.imwrite('place4.jpg', global_value.get_value('frame'))
    S.write(data2)
    time.sleep(1.8)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(0.8)
    S.write(data6)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #2
    frame = global_value.get_value('frame')
    x, y, mask = DW5(frame, ys[4])
    while MoveOrStatic_2(ys[4]) == 0 or x < 203 or x > 311 or np.sum(mask) > 3000000:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y, mask = DW5(frame, ys[4])
    print('mask5: ', np.sum(mask))
    cv2.imwrite('place5.jpg', global_value.get_value('frame'))
    S.write(data2)
    time.sleep(1.8)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(0.8)
    S.write(data7)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    see_zp()
    time.sleep(2)
    #3
    frame = global_value.get_value('frame')
    x, y, mask = DW5(frame, ys[5])
    while MoveOrStatic_2(ys[5]) == 0 or x < 203 or x > 311 or np.sum(mask) > 3000000:
        time.sleep(0.1)
        frame = global_value.get_value('frame')
        x, y, mask = DW5(frame, ys[5])
    print('mask6: ', np.sum(mask))
    cv2.imwrite('place6.jpg', global_value.get_value('frame'))
    S.write(data2)
    time.sleep(1.8)
    S.write(data14)
    time.sleep(0.5)
    BIZHI()
    time.sleep(1)
    CHUSHI()
#######################################################
#放置物料（打靶）
def Place_wl():
    #1
    S.write(data5)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[0] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[0] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    #2
    GUODU()
    time.sleep(0.3)
    S.write(data6)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[1] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[1] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    #3
    GUODU()
    time.sleep(0.3)
    S.write(data7)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[2] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[2] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
def Place_wl_2():
    #1
    S.write(data5)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[3] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[3] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    #2
    GUODU()
    time.sleep(0.3)
    S.write(data6)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[4] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[4] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    #3
    GUODU()
    time.sleep(0.3)
    S.write(data7)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[5] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[5] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
def Place_wl_3():
    #1
    S.write(data5)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[3] == 'b':
        S.write(data13)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[3] == 'r':
        S.write(data12)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data11)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    #2
    GUODU()
    time.sleep(0.3)
    S.write(data6)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[4] == 'b':
        S.write(data13)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[4] == 'r':
        S.write(data12)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data11)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    #3
    GUODU()
    time.sleep(0.3)
    S.write(data7)
    time.sleep(2.5)
    S.write(data15)
    time.sleep(0.5)
    if ys[5] == 'b':
        S.write(data13)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    elif ys[5] == 'r':
        S.write(data12)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
    else:
        S.write(data11)
        time.sleep(2.8)
        S.write(data14)
        time.sleep(0.5)
#######################################################
#加工完取回物料
def GetBack_wl():
    #1
    GUODU()
    time.sleep(0.3)
    if ys[0] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif ys[0] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data5)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #2
    if ys[1] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif ys[1] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data6)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #3
    if ys[2] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif ys[2] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data7)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
def GetBack_wl_2():
   #1
    GUODU()
    time.sleep(0.3)
    if ys[3] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif ys[3] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data5)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #2
    if ys[4] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif ys[4] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data6)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #3
    if ys[5] == 'b':
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif ys[5] == 'r':
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data7)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
def GetBack_wl_3():  #抓决赛第二层物料
    #1
    GUODU()
    time.sleep(0.3)
    if order2.index(ys[3]) == 0:
        S.write(data13)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif order2.index(ys[3]) == 2:
        S.write(data12)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data11)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data5)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #2
    if order2.index(ys[4]) == 0:
        S.write(data13)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif order2.index(ys[4]) == 2:
        S.write(data12)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data11)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data6)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #3
    if order2.index(ys[5]) == 0:
        S.write(data13)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif order2.index(ys[5]) == 2:
        S.write(data12)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data11)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data7)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
def GetBack_wl_4():  #抓决赛第一层物料
    #1
    GUODU()
    time.sleep(0.3)
    if order1.index(ys[0]) == 0:
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif order1.index(ys[0]) == 2:
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data5)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #2
    if order1.index(ys[1]) == 0:
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif order1.index(ys[1]) == 2:
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data6)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
    #3
    if order1.index(ys[2]) == 0:
        S.write(data10)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    elif order1.index(ys[2]) == 2:
        S.write(data9)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    else:
        S.write(data8)
        time.sleep(2.8)
        S.write(data15)
        time.sleep(0.5)
    GUODU()
    time.sleep(0.3)
    S.write(data7)
    time.sleep(2.5)
    S.write(data14)
    time.sleep(0.5)
#######################################################
def wl_order(frame): #计算物料顺序
    global order1
    global order2
    temp = frame[150:-1]
    g, _ = ColorRecognition('g', temp)
    r, _ = ColorRecognition('r', temp)
    b, _ = ColorRecognition('b', temp)
    if g < r and g < b:
        order1.append('g')
    elif r < g and r < b:
        order1.append('r')
    elif b < r and b < g:
        order1.append('b')
    if b < g < r or r < g < b:
        order1.append('g')
    elif b < r < g or g < r < b:
        order1.append('r')
    elif g < b < r or r < b < g:
        order1.append('b')
    if r < g and b < g:
        order1.append('g')
    elif g < r and b < r:
        order1.append('r')
    elif g < b and r < b:
        order1.append('b')
    temp2 = frame[0:150]
    g2, _ = ColorRecognition('g', temp2)
    r2, _ = ColorRecognition('r', temp2)
    b2, _ = ColorRecognition('b', temp2)
    if g2 < r2 and g2 < b2:
        order2.append('g')
    elif r2 < g2 and r2 < b2:
        order2.append('r')
    elif b2 < r2 and b2 < g2:
        order2.append('b')
    if b2 < g2 < r2 or r2 < g2 < b2:
        order2.append('g')
    elif b2 < r2 < g2 or g2 < r2 < b2:
        order2.append('r')
    elif g2 < b2 < r2 or r2 < b2 < g2:
        order2.append('b')
    if r2 < g2 and b2 < g2:
        order2.append('g')
    elif g2 < r2 and b2 < r2:
        order2.append('r')
    elif g2 < b2 and r2 < b2:
        order2.append('b')
#######################################################
#定义进程
IMG = threading.Thread(target=getFrame)
GO = threading.Thread(target=goForward)
TLY = threading.Thread(target=__tly__)
NEW_TLY = threading.Thread(target=__TLY__)
#######################################################
#开启进程
# TLY.start()
NEW_TLY.start()
IMG.start()
GO.start()
#######################################################
print('ready')
Page_pm(0)
while True:
    a = Read_pm()
    time.sleep(0.01)
    if a == 1: 
        BIZHI()
        time.sleep(1.5)
        CHUSHI()
        MCU.write(zero)
        while True:
            a = Read_pm()
            time.sleep(0.01)
            if a == 1:  #初赛
                #初赛
                ZP_limtX = 116 #118
                ZP_limtY = 154 #159
                CJG_limtX = 229
                CJG_limtY = 183
                JJG_limtX = 217
                JJG_limtY = 165
                HJ_limtX = 118
                HJ_limtY = 321
                #刷新屏幕
                Page_pm(2)
                for i in range(6):
                    display_num(i, 0)#屏幕清零
                    time.sleep(0.1)
                #获取各个角度
                start_z = None   
                while start_z == None:  #初始化角度Z，否则会返回None
                    start_z = get_new_jd(2)
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
                #初始化扫码数组
                CHUSHI()
                data = []  #存放扫码数据
                ys = [] #存放扫码数据
                #准备出发
                pin_init()#初始化各个引脚
                S.write(data14)#打开手爪
                #向左前走移动一段距离
                goForward_flag = 6
                time.sleep(1.1*t)
                goForward_flag = 1
                #向前走扫码
                move_time(1.2*t)
                BIZHI()
                time.sleep(1)
                SAOMA()#看二维码动作
                #扫码
                while len(ys) == 0:
                    frame = global_value.get_value('frame')
                    get_qr_data(frame)
                    if len(data) != 0:
                        for i in range(3):
                            display_num(i, int(data[i]))
                        for i in range(3,6):
                            display_num(i, int(data[i+1]))
                        BIZHI()
                        time.sleep(0.5)
                        DINGWEI()
                #向前走
                move_time(3.2*t)
                time.sleep(0.2)
                #转盘定位
                super_adjust(ZP_limtX, ZP_limtY)
                #抓盘区第一次抓物料
                see_zp()
                time.sleep(1.5)
                Grab_zhuanpan()
                #去粗加工区
                move_time(0.8*t)
                approximate_ToAngle(second_z)
                #更新角度,向前走
                start_z = second_z
                move_time(2.6*t)
                #转过来定位打靶
                BIZHI()
                time.sleep(1)
                S.write(data1)
                approximate_ToAngle(first_z)
                new_ToAngle(first_z)
                start_z = first_z
                #定位
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(start_z)
                super_adjust2(CJG_limtX, CJG_limtY)
                #放置物料
                Place_wl()
                #取回物料
                GetBack_wl()
                BIZHI()
                time.sleep(1)
                CHUSHI()
                #去精加工区
                approximate_ToAngle(second_z)
                start_z = second_z
                move_time(1.95*t) #2.05
                approximate_ToAngle(third_z)
                start_z = third_z
                move_time(2.2*t)
                BIZHI()
                time.sleep(1)
                S.write(data1)
                #定位
                approximate_ToAngle(second_z)
                new_ToAngle(second_z)
                start_z = second_z
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(start_z)
                super_adjust2(CJG_limtX, CJG_limtY)
                #放置物料
                Place_wl()
                GUODU()
                time.sleep(1)
                BIZHI()
                time.sleep(1)
                CHUSHI()
                #返回原料区
                approximate_ToAngle(first_z)
                start_z = first_z
                move_time(2.5*t)
                approximate_ToAngle(fouth_z)
                start_z = fouth_z
                move_time(4.4*t)
                approximate_ToAngle(first_z)
                start_z = first_z
                BIZHI()
                time.sleep(1)
                DINGWEI()
                move_time_2(0.8*t)
                time.sleep(0.2)
                #定位
                super_adjust(ZP_limtX, ZP_limtY)
                #抓物料
                see_zp()
                time.sleep(1.5)
                Grab_zhuanpan_2()
                #去粗加工区
                move_time(0.8*t)
                approximate_ToAngle(second_z)
                #更新角度
                start_z = second_z
                move_time(2.7*t)
                BIZHI()
                time.sleep(1)
                S.write(data1)
                approximate_ToAngle(first_z)
                new_ToAngle(first_z)
                start_z = first_z
                #定位
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(start_z)
                super_adjust2(CJG_limtX, CJG_limtY)
                #放置物料
                Place_wl_2()
                #取回物料
                GetBack_wl_2()
                BIZHI()
                time.sleep(1.0)
                CHUSHI()
                #去精加工区
                approximate_ToAngle(second_z)
                start_z = second_z
                move_time(2.0*t)
                approximate_ToAngle(third_z)
                start_z = third_z
                move_time(2.2*t)
                #码垛物料定位
                BIZHI()
                time.sleep(1)
                S.write(data1)
                approximate_ToAngle(second_z)
                new_ToAngle(second_z)
                start_z = second_z
                adjust3(JJG_limtX, JJG_limtY)
                new_ToAngle(start_z)
                super_adjust3(JJG_limtX, JJG_limtY)
                #放置物料
                Place_wl_3()
                GUODU()
                time.sleep(1.0)
                BIZHI()
                time.sleep(1.0)
                CHUSHI()
                #回家
                approximate_ToAngle(third_z)
                start_z = third_z
                move_time(2.2*t)
                see_zp()
                time.sleep(1)
                #定位回家
                adjust4(HJ_limtX, HJ_limtY)
                CHUSHI()
                #回家
                goForward_flag = 7
                time.sleep(1.9*t)
                goForward_flag = 0
                GPIO.cleanup()
            elif a == 0:
                print('mode1 break')
                break
    elif a == 2:
        while True:
            time.sleep(0.01)
            a = Read_pm()
            if a == 1:
                print('mode2 run')
                #决赛方案一（扫码区不变）
                ZP_limtX = 130
                ZP_limtY = 158
                CJG_limtX = 229
                CJG_limtY = 183
                bcp_limtX_0 = 70
                bcp_limtY_0 = 180
                bcp_limtX_1 = 208
                bcp_limtY_1 = 183
                bcp_limtX_2 = 217
                bcp_limtY_2 = 164
                #刷新屏幕
                Page_pm(2)
                for i in range(6):
                    display_num(i, 0)#屏幕清零
                    time.sleep(0.1)
                #获取各个角度
                start_z = None   
                while start_z == None:  #初始化角度Z，否则会返回None
                    start_z = get_new_jd(2)
                first_z = start_z
                second_z = start_z + 90 #获取角度
                if second_z > 360:
                    second_z = second_z - 360
                third_z = start_z + 180 #获取角度
                if third_z > 360:
                    third_z =  third_z - 360
                forth_z = start_z - 90  #获取角度
                if forth_z < 0:
                    forth_z =  360 + forth_z
                #初始化扫码数组
                CHUSHI()
                data = []  #存放扫码数据
                ys = [] #存放扫码数据
                #准备出发
                pin_init()#初始化各个引脚
                S.write(data14)#打开手爪
                #向右前移动一段距离
                goForward_flag = 7
                time.sleep(1.0*t)
                goForward_flag = 1
                #去扫码
                approximate_ToAngle(forth_z)
                start_z = forth_z
                move_time(5.0*t)
                approximate_ToAngle(first_z)
                start_z = first_z
                move_time(1.5*t)
                #扫码动作组
                BIZHI()
                time.sleep(1)
                SAOMA()#看二维码动作
                #扫码
                while len(ys) == 0:
                    frame = global_value.get_value('frame')
                    get_qr_data(frame)
                    if len(data) != 0:
                        for i in range(3):
                            display_num(i, int(data[i]))
                        for i in range(3,6):
                            display_num(i, int(data[i+1]))
                        BIZHI()
                        time.sleep(1)
                        CHUSHI()
                #后退
                move_time_2(1.0*t)
                #去半成品区
                approximate_ToAngle(second_z)
                start_z = second_z
                move_time(4.3*t)
                approximate_ToAngle(first_z)
                start_z = first_z
                move_time(2.5*t)
                BIZHI()
                time.sleep(0.5)
                S.write(data1)
                approximate_ToAngle(second_z)
                new_ToAngle(second_z)
                start_z = second_z
                #在半成品区粗调整位置看顺序
                CU_DINGWEI()
                time.sleep(0.5)
                adjust5(bcp_limtX_0, bcp_limtY_0)
                wl_order(global_value.get_value('frame'))
                print(order1)
                print(order2)
                #在半成品区调整位置1
                S.write(data1)
                adjust6(bcp_limtX_1, bcp_limtY_1, order1[1])
                new_ToAngle(second_z)
                super_adjust6(bcp_limtX_1, bcp_limtY_1, order1[1])
                #抓取物料
                GetBack_wl_4()
                BIZHI()
                time.sleep(0.5)
                CHUSHI()
                #去粗加工区
                approximate_ToAngle(first_z)
                start_z = first_z
                move_time(2.2*t)
                approximate_ToAngle(forth_z)
                start_z = forth_z
                move_time(2.2*t)
                BIZHI()
                time.sleep(0.5)
                S.write(data1)
                approximate_ToAngle(first_z)
                start_z = first_z
                new_ToAngle(first_z)
                #在粗加工区定位
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(first_z)
                super_adjust2(CJG_limtX, CJG_limtY)
                #打靶
                Place_wl()
                #取回物料
                GetBack_wl()
                BIZHI()
                time.sleep(0.5)
                CHUSHI()
                #去原料区
                approximate_ToAngle(forth_z)
                start_z = forth_z
                move_time(2.6*t)
                approximate_ToAngle(first_z)
                start_z = first_z
                BIZHI()
                time.sleep(0.5)
                DINGWEI()
                move_time_2(0.7*t)
                time.sleep(0.2)
                #在原料区定位
                super_adjust(ZP_limtX, ZP_limtY)
                #在原料区打靶
                Place_wl_4()
                BIZHI()
                time.sleep(1)
                CHUSHI()
                #去半成品区
                move_time(0.7*t)
                approximate_ToAngle(second_z)
                start_z = second_z #更新角度,向前走
                move_time(4.9*t)
                approximate_ToAngle(third_z)
                start_z = third_z
                move_time(2.2*t)
                approximate_ToAngle(second_z)
                new_ToAngle(second_z)
                start_z = second_z
                BIZHI()
                time.sleep(1)
                JUESAI_DINGWEI()
                time.sleep(0.5)
                #在半成品区调整
                adjust6(bcp_limtX_2, bcp_limtY_2, order2[1])
                new_ToAngle(second_z)
                super_adjust6(bcp_limtX_2, bcp_limtY_2, order2[1])
                #抓取第二批物料
                GetBack_wl_3()
                BIZHI()
                time.sleep(1)
                CHUSHI()
                #去粗加工区
                approximate_ToAngle(first_z)
                start_z = first_z
                move_time(2.2*t)
                approximate_ToAngle(forth_z)
                start_z = forth_z
                move_time(2.4*t)
                BIZHI()
                time.sleep(0.5)
                S.write(data1)
                approximate_ToAngle(first_z)
                start_z = first_z
                new_ToAngle(first_z)
                #在粗加工区定位
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(first_z)
                super_adjust2(CJG_limtX, CJG_limtY)
                #打靶
                Place_wl_2()
                #取回物料
                GetBack_wl_2()
                BIZHI()
                time.sleep(0.5)
                CHUSHI()
                #去原料区
                approximate_ToAngle(forth_z)
                start_z = forth_z
                move_time(2.6*t)
                approximate_ToAngle(first_z)
                start_z = first_z
                BIZHI()
                time.sleep(0.5)
                DINGWEI()
                move_time_2(0.7*t)
                time.sleep(0.2)
                #在原料区定位
                super_adjust(ZP_limtX, ZP_limtY)
                #在原料区打靶
                Place_wl_5()
                BIZHI()
                time.sleep(1)
                CHUSHI()
            elif a == 0:
                print('mode2 break')
                break
    elif a == 3:
        while True:
            time.sleep(0.01)
            a = Read_pm()
            if a == 1:
                #test
                ZP_limtX = 118 #118
                ZP_limtY = 159 #159
                CJG_limtX = 229
                CJG_limtY = 183
                JJG_limtX = 217
                JJG_limtY = 165
                HJ_limtX = 118
                HJ_limtY = 321
                #刷新屏幕
                Page_pm(2)
                for i in range(6):
                    display_num(i, 0)#屏幕清零
                    time.sleep(0.1)
                #获取各个角度
                start_z = None   
                while start_z == None:  #初始化角度Z，否则会返回None
                    start_z = get_new_jd(2)
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
                #初始化扫码数组
                CHUSHI()
                data = []  #存放扫码数据
                ys = [] #存放扫码数据
                #准备出发
                pin_init()#初始化各个引脚
                S.write(data14)#打开手爪
                #向左前走移动一段距离
                goForward_flag = 6
                time.sleep(1.1*t)
                goForward_flag = 1
                #向前走扫码
                move_time(1.2*t)
                BIZHI()
                time.sleep(1)
                SAOMA()#看二维码动作
                #扫码
                while len(ys) == 0:
                    frame = global_value.get_value('frame')
                    get_qr_data(frame)
                    if len(data) != 0:
                        for i in range(3):
                            display_num(i, int(data[i]))
                        for i in range(3,6):
                            display_num(i, int(data[i+1]))
                        BIZHI()
                        time.sleep(0.5)
                        DINGWEI()
                #向前走
                move_time(3.2*t)
                time.sleep(0.2)
                #转盘定位
                adjust(ZP_limtX, ZP_limtY)
                #抓盘区第一次抓物料
                CHUSHI()
                #去粗加工区
                move_time(0.8*t)
                approximate_ToAngle(second_z)
                #更新角度,向前走
                start_z = second_z
                move_time(2.6*t)
                #转过来定位打靶
                BIZHI()
                time.sleep(1)
                S.write(data1)
                approximate_ToAngle(first_z)
                new_ToAngle(first_z)
                start_z = first_z
                #定位
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(start_z)
                #放置物料
                #取回物料
                BIZHI()
                time.sleep(0.5)
                CHUSHI()
                #去精加工区
                approximate_ToAngle(second_z)
                start_z = second_z
                move_time(2.05*t)
                approximate_ToAngle(third_z)
                start_z = third_z
                move_time(2.2*t)
                BIZHI()
                time.sleep(1)
                S.write(data1)
                #定位
                approximate_ToAngle(second_z)
                new_ToAngle(second_z)
                start_z = second_z
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(start_z)
                #放置物料
                BIZHI()
                time.sleep(0.5)
                CHUSHI()
                #返回原料区
                approximate_ToAngle(first_z)
                start_z = first_z
                move_time(2.5*t)
                approximate_ToAngle(fouth_z)
                start_z = fouth_z
                move_time(4.4*t)
                approximate_ToAngle(first_z)
                start_z = first_z
                BIZHI()
                time.sleep(1)
                DINGWEI()
                move_time_2(0.8*t)
                time.sleep(0.2)
                #定位
                adjust(ZP_limtX, ZP_limtY)
                #抓物料
                CHUSHI()
                #去粗加工区
                move_time(0.8*t)
                approximate_ToAngle(second_z)
                #更新角度
                start_z = second_z
                move_time(2.7*t)
                BIZHI()
                time.sleep(1)
                S.write(data1)
                approximate_ToAngle(first_z)
                new_ToAngle(first_z)
                start_z = first_z
                #定位
                adjust2(CJG_limtX, CJG_limtY)
                new_ToAngle(start_z)
                super_adjust2(CJG_limtX, CJG_limtY)
                #放置物料
                #取回物料
                BIZHI()
                time.sleep(1.0)
                CHUSHI()
                #去精加工区
                approximate_ToAngle(second_z)
                start_z = second_z
                move_time(2.0*t)
                approximate_ToAngle(third_z)
                start_z = third_z
                move_time(2.2*t)
                #码垛物料定位
                BIZHI()
                time.sleep(1)
                S.write(data1)
                approximate_ToAngle(second_z+1)
                new_ToAngle(second_z+1)
                start_z = second_z+1
                adjust3(JJG_limtX, JJG_limtY)
                new_ToAngle(start_z)
                #放置物料
                BIZHI()
                time.sleep(1.0)
                CHUSHI()
                #回家
                approximate_ToAngle(third_z)
                start_z = third_z
                move_time(2.2*t)
                see_zp()
                time.sleep(1)
                #定位回家
                adjust4(HJ_limtX, HJ_limtY)
                CHUSHI()
                #回家
                goForward_flag = 7
                time.sleep(1.9*t)
                goForward_flag = 0
                GPIO.cleanup()
            elif a == 0:
                print('test break')
                break
    elif a == 4:
        print(4)
        while True:
            time.sleep(0.01)
            a = Read_pm()
            if a == 0:
                see_zp()
            elif a == 1:
                S.write(data1)
            elif a == 2:
                S.write(data2)
            elif a == 3:
                S.write(data3)
            elif a == 4:
                S.write(data4)
            elif a == 5:
                BIZHI()
                time.sleep(1.5)
                S.write(data5)
            elif a == 6:
                BIZHI()
                time.sleep(1.5)
                S.write(data6)
            elif a == 7:
                BIZHI()
                time.sleep(1.5)
                S.write(data7)
            elif a == 8:
                S.write(data8)
            elif a == 9:
                S.write(data9)
            elif a == 10:
                S.write(data10)
            elif a == 11:
                S.write(data11)
            elif a == 12:
                S.write(data12)
            elif Read_pm() == 13:
                S.write(data13)
            elif a == 14:
                S.write(data14)
            elif a == 15:
                S.write(data15)
            elif a == 16:
                GUODU()
                break
