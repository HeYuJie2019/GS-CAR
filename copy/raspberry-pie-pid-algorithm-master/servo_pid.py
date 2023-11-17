import RPi.GPIO as GPIO
import time


servopin=18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servopin,GPIO.OUT,initial=False)
p=GPIO.PWM(servopin,50)	#50HZ:频率就是周期脉冲的周期的倒数
p.start(0)					#start(initdutycycle)：占空比0-100间，0表示暂不输出
time.sleep(2)



try:

    while(True):

        for i in range(0,181,10):
            p.ChangeDutyCycle(2.5+10*i/180)			#设置转动角度
            time.sleep(0.02)						#等该20ms周期结束  
            p.ChangeDutyCycle(0)					#归零信号  
            time.sleep(0.2)
    
        for i in range(181,0,-10):
            p.ChangeDutyCycle(2.5+10*i/180)
            time.sleep(0.02)
            p.ChangeDutyCycle(0)
            time.sleep(0.2)

except Exception as r:
    print(r)

finally:
    p.stop()  #停止输出PWM波
    GPIO.cleanup()  #程序的最后别忘记清除所有资源





# 我觉得用r/180*10+2.5更直观一点，就是180度的角度对应百分之十的占空比，计算出角度r对应的占空比变化量，加上0度的占空比2.5，就是角度r对应的占空比
