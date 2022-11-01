#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import Float64

class Controles:

    def __init__(self):

        self.publish_time = 0.05

        self.dados = rospy.Subscriber("/dados", Float64, self.reed, queue_size = 10000)

        self.full_left = self.dados[0] 
        self.left = self.dados[1]
        self.center = self.dados[2]
        self.right = self.dados[3]
        self.full_right = self.dados[4]

        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_time), self.timerCallback)   

    def reed (self):

        if self.full_left == 0 and self.left == 0 and self.full_right == 0 and self.right == 0 and self.center == 1:
            print("frente")

        if self.full_left == 0 and self.left == 0 and self.full_right == 0 and self.right == 0 and self.center == 0:
            print("para")

        if self.full_left == 1 and self.left == 1 and self.full_right == 0 and self.right == 0 and self.center == 1:
            print("esquerda")
        
        if self.full_left == 0 and self.left == 0 and self.full_right == 1 and self.right == 1 and self.center == 1:
            print("direita")
        


    def timerCallback(self, event):
        msg = Float64()
        msg.data = self.vel
        self.sensor_pub.publish(msg)

if __name__ == '__main__':

    try:
        rospy.init_node("sensores")
        Controles()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass