import struct
import RPi.GPIO as GPIO
import time
import pigpio
import serial
import global_value
import threading
global_value._init()  #全局变量初始化，初始化字典
pi = pigpio.pi()
MCU = serial.Serial("/dev/ttyAMA1", baudrate=230400) #设置高精度陀螺仪串口

key = 0    #陀螺仪函数的全局变量
buff = {}  #陀螺仪函数的全局变量

# PID控制参数
Kp = 2                # 比例系数
Ki = 0                 # 积分系数
Kd = 0.06                # 导数系数
PID_SCALE= 0.01          #PID缩放系数 
PID_INTEGRAL_UP = 500  #积分上限

# 控制周期（秒）
control_period = 0.01

# 定义锁
global_lock = threading.Lock()

# 定义GPIO引脚
# 连接霍尔编码器的GPIO引脚
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

# 初始化GPIO
GPIO.setmode(GPIO.BCM)
GPIO_in_list = (encoder_pin_A, encoder_pin_B, encoder_pin_C, encoder_pin_D)
GPIO.setup(GPIO_in_list, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 设置计数器
pulse_count_a = 0        #编码器当前数值
pulse_count_b = 0
pulse_count_c = 0
pulse_count_d = 0
count_a = 0       
count_b = 0
count_c = 0
count_d = 0
record_encoder_a = 0     #编码器计数数值
record_encoder_b = 0    
record_encoder_c = 0     
record_encoder_d = 0 
bias_last_a = 0          #上一个偏差
bias_last_b = 0 
bias_last_c = 0 
bias_last_d = 0 
bias_integral_a = 0      #积分
bias_integral_b = 0   
bias_integral_c = 0   
bias_integral_d = 0  

record_count = record_time = 0 # 计数器, 时间器,   # 每3次计算一次速度

#陀螺仪程序
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
#陀螺仪进程  HWT101CT-TTL
def __TLY__():
    while True:
        buff_count = MCU.inWaiting()
        buff_data = MCU.read(buff_count)
        for i in range(0, buff_count):
            new_handleSerialData(buff_data[i])

# 定义中断处理函数
def pulse_callback_a(channel):
    global pulse_count_a, count_a
    if GPIO.input(encoder_pin_A) == 1:  
        pulse_count_a += 1
        count_a += 1
        
def pulse_callback_b(channel):
    global pulse_count_b, count_b
    if GPIO.input(encoder_pin_B) == 1:  
        pulse_count_b += 1
        count_b += 1
        
def pulse_callback_c(channel):
    global pulse_count_c, count_c
    if GPIO.input(encoder_pin_C) == 1:  
        pulse_count_c += 1
        count_c += 1
        
def pulse_callback_d(channel):
    global pulse_count_d, count_d
    if GPIO.input(encoder_pin_D) == 1:  
        pulse_count_d += 1
        count_d += 1

# 将中断处理函数绑定到引脚上
GPIO.add_event_detect(encoder_pin_A, GPIO.RISING, callback=pulse_callback_a)
GPIO.add_event_detect(encoder_pin_B, GPIO.RISING, callback=pulse_callback_b)
GPIO.add_event_detect(encoder_pin_C, GPIO.RISING, callback=pulse_callback_c)
GPIO.add_event_detect(encoder_pin_D, GPIO.RISING, callback=pulse_callback_d)

# 计算电机转速的函数
def get_spd():
    global record_count, record_time, pulse_count_a, pulse_count_b, pulse_count_c, pulse_count_d, record_encoder_a, record_encoder_b, record_encoder_c, record_encoder_d
    if record_count == 3:
        current_time = round(time.time()*1000)
        
        # 电机编码器的计数差
        dc_a = pulse_count_a - record_encoder_a
        record_encoder_a = pulse_count_a = 0
        
        dc_b = pulse_count_b - record_encoder_b
        record_encoder_b = pulse_count_b = 0
        
        dc_c = pulse_count_c - record_encoder_c
        record_encoder_c = pulse_count_c = 0
        
        dc_d = pulse_count_d - record_encoder_d
        record_encoder_d = pulse_count_d = 0
        
        # 更新参数
        dt = current_time - record_time
        record_time = current_time
        record_count = 0
        
        spd_a = dc_a / dt
        spd_b = dc_b / dt
        spd_c = dc_c / dt
        spd_d = dc_d / dt
        
        return spd_a, spd_b, spd_c, spd_d
    else:
        record_count += 1
    return None, None, None, None
def GetSpeed():
    try:
        while True:
            # 获取电机转速
            speed_a, speed_b, speed_c, speed_d = get_spd()
            time.sleep(0.02)
            
            if speed_a != None and speed_b != None and speed_c != None and speed_d != None:
                speed_a = speed_a * 100
                speed_b = speed_b * 100
                speed_c = speed_c * 100
                speed_d = speed_d * 100
                with global_lock:
                    global_value.set_value('motorA', speed_a)
                    global_value.set_value('motorB', speed_b)
                    global_value.set_value('motorC', speed_c)
                    global_value.set_value('motorD', speed_d)
                    global_value.set_value('motorA_', count_a)
                    global_value.set_value('motorB_', count_b)
                    global_value.set_value('motorC_', count_c)
                    global_value.set_value('motorD_', count_d)
                
            buff_count = MCU.inWaiting()
            buff_data = MCU.read(buff_count)
            # print(buff_data)
            for i in range(0, buff_count):
                new_handleSerialData(buff_data[i])
            # print(global_value.get_value('JD'))
            # print("a:" ,global_value.get_value('motorA'), "  b:" ,global_value.get_value('motorB'), "  c:" ,global_value.get_value('motorC'),  "  d:" ,global_value.get_value('motorD'))
    except KeyboardInterrupt:
        pass
    # 清理GPIO资源
    GPIO.cleanup()
def SpeedControl_A():
    global bias_integral_a, bias_last_a
    target_speed = global_value.set_value('targetA', 0)
    current_speed = global_value.set_value('motorA', 0)
    output = 0
    
    while True:
        if global_value.get_value('model') == 0:
            target_speed = global_value.get_value('targetA')
            # 获取当前电机速度
            current_speed = global_value.get_value('motorA')  # 这里需要根据实际情况获取电机速度的函数
            # 获得偏差值
            bias = target_speed - current_speed
            # 计算偏差累加值
            bias_integral_a += bias
            # 抗积分饱和
            if(bias_integral_a > PID_INTEGRAL_UP): bias_integral_a = PID_INTEGRAL_UP
            if(bias_integral_a < -PID_INTEGRAL_UP): bias_integral_a = -PID_INTEGRAL_UP
            # PID计算电机输出PWM值
            output += Kp*bias*PID_SCALE + Kd*(bias-bias_last_a)*PID_SCALE + Ki*bias_integral_a*PID_SCALE
            # 记录上次偏差
            bias_last_a = bias
            # 限制输出范围
            if output > 50:
                output = 50
            elif output < 0:
                output = 0
            # 将输出应用于电机控制
            num = int(40000 * output * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMA, num)
            # 等待控制周期
            time.sleep(control_period)
        elif global_value.get_value('model') == 1:
            target_speed = global_value.get_value('targetA')
            num = int(40000 * target_speed * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMA, num)
def SpeedControl_B():
    global bias_integral_b, bias_last_b
    target_speed = global_value.set_value('targetB', 0)
    current_speed = global_value.set_value('motorB', 0)
    output = 0
    
    while True:
        if global_value.get_value('model') == 0:
            target_speed = global_value.get_value('targetB')
            # 获取当前电机速度
            current_speed = global_value.get_value('motorB')  # 这里需要根据实际情况获取电机速度的函数
            # 获得偏差值
            bias = target_speed - current_speed
            # 计算偏差累加值
            bias_integral_b += bias
            # 抗积分饱和
            if(bias_integral_b > PID_INTEGRAL_UP): bias_integral_b = PID_INTEGRAL_UP
            if(bias_integral_b < -PID_INTEGRAL_UP): bias_integral_b = -PID_INTEGRAL_UP
            # PID计算电机输出PWM值
            output += Kp*bias*PID_SCALE + Kd*(bias-bias_last_b)*PID_SCALE + Ki*bias_integral_b*PID_SCALE
            # 记录上次偏差
            bias_last_b = bias
            # 限制输出范围
            if output > 50:
                output = 50
            elif output < 0:
                output = 0
            # 将输出应用于电机控制
            num = int(40000 * output * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMB, num)
            # 等待控制周期
            time.sleep(control_period)
        elif global_value.get_value('model') == 1:
            target_speed = global_value.get_value('targetB')
            num = int(40000 * target_speed * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMB, num)
def SpeedControl_C():
    global bias_integral_c, bias_last_c
    target_speed = global_value.set_value('targetC', 0)
    current_speed = global_value.set_value('motorC', 0)
    output = 0
    
    while True:
        if global_value.get_value('model') == 0:
            target_speed = global_value.get_value('targetC')
            # 获取当前电机速度
            current_speed = global_value.get_value('motorC')  # 这里需要根据实际情况获取电机速度的函数
            # 获得偏差值
            bias = target_speed - current_speed
            # 计算偏差累加值
            bias_integral_c += bias
            # 抗积分饱和
            if(bias_integral_c > PID_INTEGRAL_UP): bias_integral_c = PID_INTEGRAL_UP
            if(bias_integral_c < -PID_INTEGRAL_UP): bias_integral_c = -PID_INTEGRAL_UP
            # PID计算电机输出PWM值
            output += Kp*bias*PID_SCALE + Kd*(bias-bias_last_c)*PID_SCALE + Ki*bias_integral_c*PID_SCALE
            # 记录上次偏差
            bias_last_c = bias
            # 限制输出范围
            if output > 50:
                output = 50
            elif output < 0:
                output = 0
            # 将输出应用于电机控制
            num = int(40000 * output * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMC, num)
            # 等待控制周期
            time.sleep(control_period)
        elif global_value.get_value('model') == 1:
            target_speed = global_value.get_value('targetC')
            num = int(40000 * target_speed * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMC, num)
def SpeedControl_D():
    global bias_integral_d, bias_last_d
    target_speed = global_value.set_value('targetD', 0)
    current_speed = global_value.set_value('motorD', 0)
    output = 0
    
    while True:
        if global_value.get_value('model') == 0:
            target_speed = global_value.get_value('targetD')
            # 获取当前电机速度
            current_speed = global_value.get_value('motorD')  # 这里需要根据实际情况获取电机速度的函数
            # 获得偏差值
            bias = target_speed - current_speed
            # 计算偏差累加值
            bias_integral_d += bias
            # 抗积分饱和
            if(bias_integral_d > PID_INTEGRAL_UP): bias_integral_d = PID_INTEGRAL_UP
            if(bias_integral_d < -PID_INTEGRAL_UP): bias_integral_d = -PID_INTEGRAL_UP
            # PID计算电机输出PWM值
            output += Kp*bias*PID_SCALE + Kd*(bias-bias_last_d)*PID_SCALE + Ki*bias_integral_d*PID_SCALE
            # 记录上次偏差
            bias_last_d = bias
            # 限制输出范围
            if output > 50:
                output = 50
            elif output < 0:
                output = 0
            # 将输出应用于电机控制
            num = int(40000 * output * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMD, num)
            # 等待控制周期
            time.sleep(control_period)
        elif global_value.get_value('model') == 1:
            target_speed = global_value.get_value('targetD')
            num = int(40000 * target_speed * 0.01)  # 占空比 0~100
            pi.set_PWM_dutycycle(PWMD, num)

# GetSpeed()