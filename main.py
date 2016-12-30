#coding:utf-8
import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep

def initiate_GPIO(channel):
    '''
    初始化GPIO 口
    :param channel: the GPIO pin 树莓派的针脚
    :return: none
    '''
    GPIO.setmode(GPIO.BCM) #set the raspi's mode as BCM
    GPIO.setwarnings(0)
    GPIO.setup(channel,GPIO.OUT)

def change_angle(img):
    '''
    Define a function that give the PWM a proper dc to adjust the
 angle of Steering gear
    给出一个合适的 dc值 到舵机角度的映射函数，通过调试发现，dc值与舵机需要转动角度映射关系为 舵机角度=dc*15
    :param img: The result of the object-detect
    :return: the  of the PWM
    '''
    a,b = get_locate(img)
    s,l = img.shape
    a = a/(l)
    dc = a*15
    if dc>0:
        return dc
    return 5

def seek_object(img,channel,frequency):
    '''

    :param img: 摄像头读取的图像
    :param channel: PWM波输出的GPIO口
    :param frequency: 舵机的频率
    :return:
    '''
    initiate_GPIO(channel)
    dc = change_angle(img)
    p = GPIO.PWM(channel,frequency) # Create a PWM instance
    p.start(dc) # To start PWM
    p.ChangeDutyCycle(dc)  #To change the duty cycle
    sleep(1)
    #p.stop() #To stop PWM


def detect_locate(img):
    '''
    以步长5遍历整个图像，检测屏幕是否出现了绿色物体
    :param img: 需要检测的图像
    :return:
    '''
    a, b, c = img.shape
    for s in range(1, a - 1, 5):
        for l in range(1, b - 1, 5):
            detect_result = img[s, l] == [0., 0., 0.]
            if detect_result[0] == False:
                return True
    return False


def get_medium(set):
    '''
    计算一个集合的中位数
    :param set: 集合
    :return:
    '''
    count=0
    sum_x=0
    for x in set:
        sum_x+=x
        count+=1
    if count == 0:
        return 0
    return sum_x/count


def get_locate(frame):
    '''
    给出图像中被检测物体的中心
    :param frame: 图像
    :return:
    '''
    a, b = frame.shape
    count = 0
    s_set = []
    l_set = []
    for s in range(1, a - 1, 5):
        for l in range(1, b - 1, 5):
            detect_result = frame[s, l] == [0., 0., 0.]
            if detect_result[0] == False:
                s_set.append(s)
                l_set.append(l)
    s_average = get_medium(s_set)
    l_average = get_medium(l_set)
    return s_average,l_average
'''
    if s_average>=b/2+10:
        print "right"
    elif s_average<=b/2:

        print 'left'
    else:
        print "Medium"
'''



def main():
    cap = cv2.VideoCapture(0)
    lower_green = np.array([35,90,46],dtype='uint8')#定义hsv空间内的绿色最低值
    upper_green = np.array([77,255,255],dtype='uint8')#定义hsv空间中绿色的最高值

    while 1:
        ret,frame = cap.read()
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)#将图像的BGR模式转化到hsv空间
        mask = cv2.inRange(hsv, lower_green, upper_green)#检测是否有绿色
        kernel = np.ones((9, 9), np.uint8) #MORPH _OPEN to remove the noisy point
        #hsv = cv2.bitwise_and(hsv,hsv,mask=mask)
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)#开运算，剔除噪点
        frame1 = cv2.bitwise_and(frame,frame,mask=mask)
        cv2.imshow('mask',mask)
        cv2.imshow('frame',frame)
        cv2.imshow('frame1',frame1)
        seek_object(mask,18,50)# 用18号GPIO口输出PWM波，频率为50hz
        if cv2.waitKey(3)&0xff==ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
