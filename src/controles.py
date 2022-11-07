#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class Controles:

    def __init__(self):

        self.vel = Twist()
        self.publish_time = 0.05
        self.angular_z = 0
        self.linear_x = 0

        rospy.Subscriber("/dados", Int32MultiArray, self.reed, queue_size = 10)

        # dados recebidos dos snesores
        self.dados = [-1, -1, -1, -1, -1]
        self.full_left = self.dados[0] 
        self.left = self.dados[1]
        self.center = self.dados[2]
        self.right = self.dados[3]
        self.full_right = self.dados[4]

        # dados enviados para o cmd_vel
        # pra esquerda angular em z negativo

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_time), self.timerCallback)   

    def reed (self, msg):

        self.dados = msg.data
        self.full_left = self.dados[0] 
        self.left = self.dados[1]
        self.center = self.dados[2]
        self.right = self.dados[3]
        self.full_right = self.dados[4]

        if self.full_left == 1 and self.left == 1 and self.full_right == 0 and self.right == 1 and self.center == 1:
            self.linear_x = 0.1

        if self.full_left == 0 and self.left == 0 and self.full_right == 0 and self.right == 0 and self.center == 0:
            print("para")

        if self.full_left == 1 and self.left == 1 and self.full_right == 0 and self.right == 0 and self.center == 1:
            print("esquerda")
        
        if self.full_left == 0 and self.left == 0 and self.full_right == 1 and self.right == 1 and self.center == 1:
            print("direita")
        


    def timerCallback(self, event):
        msg = Twist()
        msg.angular.z = self.angular_z
        msg.linear.x = self.linear_x
        self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':

    rospy.init_node("controles")
    Controles()
    rospy.spin()