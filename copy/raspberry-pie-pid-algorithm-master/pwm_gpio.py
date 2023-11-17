import RPi.GPIO as GPIO  #引入函数库

import time

channel = 11

GPIO.setmode(GPIO.BOARD) #设置引脚编号规则

GPIO.setup(channel, GPIO.OUT)  #将11号引脚设置成输出模式

p=GPIO.PWM(channel, 50)   #将11号引脚初始化为PWM实例，频率为50Hz

p.start(0)   #开始脉宽调制，参数范围为：（0.0《=dc《=100.0）

try:

    while True:
        
        for dc in range(0, 100, 5):
        
            p.ChangeDutyCycle(dc)   #修改占空比参数范围为：（0.0《=dc《=100.0）

            time.sleep(0.1)

        for dc in range(100, 0, -5):

            p.ChangeDutyCycle(dc)

            time.sleep(0.1)

except Exception as r:
    print(r)

finally:
    p.stop()  #停止输出PWM波
    GPIO.cleanup()  #程序的最后别忘记清除所有资源