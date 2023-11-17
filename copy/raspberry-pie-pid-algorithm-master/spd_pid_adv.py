import RPi.GPIO as GPIO
import time
import threading
from plot_pid import sub_plot_c


PWMA = 18          #left_pwm
AIN1   =  22       #left_in_1
AIN2   =  27       #left_in_2

SensorINPUTA = 23      #电机A相
SensorINPUTB = 24      #电机B相


PID_SCALE=  0.01      #PID缩放系数
PID_INTEGRAL_UP= 1000  #积分上限

ax_motor_kp=600;  #电机转速PID-P
ax_motor_ki=0;    #电机转速PID-I
ax_motor_kd=400;  #电机转速PID-D

global encoder_c, record_encoder, record_count, record_time  #编码器数值, 计数器, 时间器
encoder_c = record_encoder = record_count = 0 
record_time = round(time.time()*1000)  


global bias, bias_last, bias_integral, motor_pwm_out
bias = bias_last = bias_integral = motor_pwm_out = 0


draw = sub_plot_c()




GPIO.setwarnings(True) 
GPIO.setmode(GPIO.BCM)     #BCM model

GPIO.setup(AIN2,GPIO.OUT)  #total output model
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)
GPIO.setup(SensorINPUTA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SensorINPUTB,GPIO.IN, pull_up_down=GPIO.PUD_UP)


L_Motor= GPIO.PWM(PWMA, 100)
L_Motor.start(0)


# 每转一次，读取一次速度
def exti_moter1(channel):
    
    global encoder_c
    if GPIO.input(SensorINPUTB) == 1:
        encoder_c += 1

# 边缘检测
GPIO.add_event_detect(SensorINPUTB, GPIO.RISING, callback=exti_moter1)



# 通过计数1000，求得时间差dt, 读取转速差dc， 通过dc/dt求得速度
def get_spd():

    global encoder_c, record_encoder, record_count, record_time

    print('code',encoder_c)
    # print(record_count)

    if record_count == 3:
        current_time = round(time.time()*1000) 

        dc = encoder_c - record_encoder
        dt = current_time - record_time

        record_count = 0
        record_time = current_time
        record_encoder = encoder_c

        spd = dc / dt
        
        return spd
    else:
        record_count += 1

    return None
    


def t_up(speed,t_time):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, GPIO.HIGH)#AIN2
    GPIO.output(AIN1, GPIO.LOW) #AIN1
    time.sleep(t_time)


def PID_MotorCtlA(target, current):
    
    global bias,bias_last,bias_integral,motor_pwm_out

    # 获得偏差值
    bias = target - current

    # 计算偏差累加值
    bias_integral += bias

    # 抗积分饱和
    if(bias_integral>PID_INTEGRAL_UP): bias_integral = PID_INTEGRAL_UP
    if(bias_integral<-PID_INTEGRAL_UP): bias_integral = -PID_INTEGRAL_UP

    # PID计算电机输出PWM值
    motor_pwm_out += ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE
	
	# 记录上次偏差
    bias_last = bias
	
    # 限制最大输出
    if motor_pwm_out > 100:
        motor_pwm_out = 100
    if motor_pwm_out < -100:
        motor_pwm_out = -100
  
    return motor_pwm_out




def loop():
    try:

        spd_temp = 0.5
    
        while True:

            spd_cuurent = get_spd()

            if spd_cuurent: 
                spd_temp = spd_cuurent
                draw.update(y=spd_cuurent)
                draw.draw_img()
                print('spd_cuurent', spd_cuurent)

            pwm = PID_MotorCtlA(1, spd_temp)
            print('pwm', pwm)

            t_up(pwm, 0.5)   # 0.5秒改变一次

    except Exception as r:
        print(r)

    finally:
        L_Motor.stop()  #停止输出PWM波
        GPIO.cleanup()  #程序的最后别忘记清除所有资源





loop()

