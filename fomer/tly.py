import struct
import serial
import global_value
key = 0
flag = 0
buff = {}
# acceleration = [0, 0, 0] #加速度
# angle_degree = [0, 0, 0] #欧拉角
global_value._init()
# global_value.set_value('jsd',acceleration)
# global_value.set_value('jd',angle_degree)


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
        if angle_flag:
            print(
    '''
    acceleration(m/s**2):
        x:%.2f
        y:%.2f
        z:%.2f
        
    angle_degree():
        x:%.2f
        y:%.2f
        z:%.2f
    ''' % (global_value.get_value('jsd')[0], global_value.get_value('jsd')[1], global_value.get_value('jsd')[2], 
        global_value.get_value('jd')[0], global_value.get_value('jd')[1], global_value.get_value('jd')[2]+180
        ))


wt_imu = serial.Serial("/dev/ttyAMA1", baudrate=115200)
while True:
    buff_count = wt_imu.inWaiting()
    buff_data = wt_imu.read(buff_count)
    for i in range(0, buff_count):
        handleSerialData(buff_data[i])
        
        
