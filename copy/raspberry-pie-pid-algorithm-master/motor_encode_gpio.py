import RPi.GPIO as GPIO
import time


PWMA = 18          #left_pwm
AIN1   =  22       #left_in_1
AIN2   =  27       #left_in_2

SensorINPUTA = 23      #电机A相
SensorINPUTB = 24      #电机B相

global turn_count
turn_count = 0

GPIO.setwarnings(True) 
GPIO.setmode(GPIO.BCM)     #BCM model

GPIO.setup(AIN2,GPIO.OUT)  #total output model
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)
GPIO.setup(SensorINPUTA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SensorINPUTB,GPIO.IN, pull_up_down=GPIO.PUD_UP)


def exti_moter1(channel):
    if GPIO.input(SensorINPUTB) == 1:
        global turn_count
        turn_count += 1


# 边缘检测
GPIO.add_event_detect(SensorINPUTB, GPIO.RISING, callback=exti_moter1)


L_Motor= GPIO.PWM(PWMA, 100)
L_Motor.start(0)


def t_up(speed,t_time):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, GPIO.HIGH)#AIN2
    GPIO.output(AIN1, GPIO.LOW) #AIN1
    time.sleep(t_time)
        
def t_stop(t_time):
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2, GPIO.LOW)#AIN2
    GPIO.output(AIN1, GPIO.LOW) #AIN1
    time.sleep(t_time)
        
def t_down(speed,t_time):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, GPIO.LOW)#AIN2
    GPIO.output(AIN1, GPIO.HIGH) #AIN1
    time.sleep(t_time)



try:

    while True:
        
        t_up(20, 2)
        t_stop(2)
        print('完成一次')
        print(turn_count)

except Exception as r:
    print(r)

finally:
    L_Motor.stop()  #停止输出PWM波
    GPIO.cleanup()  #程序的最后别忘记清除所有资源

