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
        self.aux = 0

        # dados enviados para o cmd_vel
        # pra esquerda angular em z negativo
        self.angular_z = 0
        self.linear_x = 0

        # dados recebidos dos snesores
        self.dados = [-1, -1, -1, -1, -1]
        self.full_left = self.dados[0] 
        self.left = self.dados[1]
        self.center = self.dados[2]
        self.right = self.dados[3]
        self.full_right = self.dados[4]

        rospy.Subscriber("/dados", Int32MultiArray, self.reed, queue_size = 10)
        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_time), self.timerCallback)  

        rospy.logwarn("Rodando") 
 

    def reed (self, msg):

        self.dados = msg.data
        self.full_left = self.dados[0] 
        self.left = self.dados[1]
        self.center = self.dados[2]
        self.right = self.dados[3]
        self.full_right = self.dados[4]

        if self.aux == 1:
            self.linear_x = 0.18
            self.angular_z = -1.1
            print("virando")
            if self.full_left == 0 and self.left == 0 and self.full_right == 0 and self.right == 0 and self.center == 0:
                self.aux = 0
        else:
            #frente
            if self.full_left == 1 and self.left == 1 and self.full_right == 1 and self.right == 1 and self.center == 0:
                self.linear_x = 0.2
                print("frente")
            #cruzamento
            elif self.full_left == 0 and self.left == 0 and self.full_right == 0 and self.right == 0 and self.center == 0:
                self.linear_x = 0.2
                print("cruzamento")
            #curva 1
            elif self.full_left == 1 and self.left == 0 and self.full_right == 1 and self.right == 1 and self.center == 0:
                self.aux = 1 
                print("add 1")
            #parado
            else:
                self.linear_x = 0
                self.angular_z = 0
                print("parado")
        


    def timerCallback(self, event):
        msg = Twist()
        msg.angular.z = self.angular_z
        msg.linear.x = self.linear_x
        self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':

    rospy.init_node("controles")
    Controles()
    rospy.spin()