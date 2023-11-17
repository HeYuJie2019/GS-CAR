import serial
import struct
import threading
import time
import global_value
global_value._init()  #全局变量初始化，初始化字典
key = 0    #陀螺仪函数的全局变量
buff = {}  #陀螺仪函数的全局变量
wt_imu = serial.Serial("/dev/ttyAMA1", baudrate=115200) #设置陀螺仪串口
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
def __tly__():
    while True:
        buff_count = wt_imu.inWaiting()
        buff_data = wt_imu.read(buff_count)
        for i in range(0, buff_count):
            handleSerialData(buff_data[i])
#返回角度
def get_jd(axis,defValue=None):  
    try :
        return global_value.get_value('jd')[axis]+180
    except TypeError:
        return defValue

TLY = threading.Thread(target=__tly__)
TLY.start()

start_z = None   
while start_z == None:  #初始化角度Z，否则会返回None
    start_z = get_jd(2)

num = 0
while True:
    a = get_jd(2)
    time.sleep(0.001)
    b = get_jd(2)
    num = num + b - a
    print(num)