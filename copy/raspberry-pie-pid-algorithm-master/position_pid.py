import RPi.GPIO as GPIO
import time
import threading


PWMA = 18          #left_pwm
AIN1   =  22       #left_in_1
AIN2   =  27       #left_in_2

SensorINPUTA = 23      #电机A相
SensorINPUTB = 24      #电机B相


GPIO.setwarnings(True) 
GPIO.setmode(GPIO.BCM)     #BCM model

GPIO.setup(AIN2,GPIO.OUT)  #total output model
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)
GPIO.setup(SensorINPUTA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SensorINPUTB,GPIO.IN, pull_up_down=GPIO.PUD_UP)


# 速度最大值，当前速度
PWM_MAX = 100



# 积分、微分偏差
global Integral_bias, Last_Bias, spwm
Integral_bias, Last_Bias, spwm = 0, 0, 0


# 后面可以手动控制pid的值
global Position_KP, Position_KI, Position_KD
Position_KP = 2    # 50/2300
Position_KI = 0.05
Position_KD = 0.8


# 这个目标速度是人为设置的， 为的是让马达符合设置的速度
global encoder_c, speed_v
target_v = 1.7
encoder_c = 0
speed_v = 0       # pwm为50时, 2转/毫秒


# 通过累加量/时间 求得速度   转/秒
global nw_time
st_time = round(time.time()*1000)

# 计算速度
def get_speed():
    nw_time = round(time.time()*1000)
    global encoder_c, st_time, speed_v
    speed_v = 0 if nw_time - st_time == 0 else encoder_c / (nw_time - st_time)




# 每转一次，读取一次速度
def exti_moter1(channel):
    
    global encoder_c
    if GPIO.input(SensorINPUTB) == 1:
        encoder_c += 1
    else:
        encoder_c -= 1



# 边缘检测
GPIO.add_event_detect(SensorINPUTB, GPIO.RISING, callback=exti_moter1)



# B相每计数一次，执行一次PID
def Position_PID(Encoder_v, Target_v):

    global Integral_bias, Last_Bias, spwm

    Bias = Encoder_v - Target_v

    Integral_bias += Bias

    spwm = spwm - ( Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias-Last_Bias) )

    Last_Bias = Bias



def t_up(speed,t_time):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, GPIO.HIGH)#AIN2
    GPIO.output(AIN1, GPIO.LOW) #AIN1
    time.sleep(t_time)


# 开启一个新的守护线程，执行PWM转动
def PID_RUN():
    while True:

        global encoder_c, spwm, speed_v
        get_speed()

        Position_PID(speed_v, target_v)

        spwm = PWM_MAX if spwm > PWM_MAX else spwm
        spwm = 0 if spwm < 0 else spwm

        t_up(spwm, 0.5)   # 0.5秒改变一次



L_Motor= GPIO.PWM(PWMA, 100)
L_Motor.start(0)


servo_tid=threading.Thread(target=PID_RUN)  # 多线程
servo_tid.setDaemon(True)
servo_tid.start()                               # 开启线程


try:

    while True:

        nw_time = round(time.time()*1000)
        

        print('pid', Integral_bias, Last_Bias)
        print('pwm', spwm)
        
        print('encoder_c', encoder_c)
        print('time_count', nw_time - st_time)
        print('speed', speed_v)

        print("##################################################")

except Exception as r:
    print(r)

finally:
    L_Motor.stop()  #停止输出PWM波
    GPIO.cleanup()  #程序的最后别忘记清除所有资源


