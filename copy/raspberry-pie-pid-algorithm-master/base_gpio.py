import RPi.GPIO as GPIO  #引入函数库

import time

channel = 11

GPIO.setmode(GPIO.BOARD) #设置引脚编号规则

GPIO.setup(channel, GPIO.OUT)  #将11号引脚设置成输出模式

try:

    while True:

        GPIO.output(channel, GPIO.HIGH)  #将引脚的状态设置为高电平，此时LED亮了

        time.sleep(1)  #程序休眠1秒钟，让LED亮1秒

        GPIO.output(channel, GPIO.LOW)  #将引脚状态设置为低电平，此时LED灭了

        time.sleep(1)   #程序休眠1秒钟，让LED灭1秒

except Exception as r:
    print(r)

finally:
    GPIO.cleanup()  #程序的最后别忘记清除所有资源