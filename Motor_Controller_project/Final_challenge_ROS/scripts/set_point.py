#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int32
from std_msgs.msg import String
from final_challenge.msg import motor_output
import numpy as np

k = 'S'

def shutdown():
    shutdown_msg = motor_output()
    shutdown_msg.vel = 0
    shutdown_msg.direction = "forward"
    pub.publish(shutdown_msg)

def keyd_callback(msg):
   global k
   k = msg.data
   rospy.loginfo(k)

if __name__ == '__main__':
    rospy.init_node("set_point")
    pub = rospy.Publisher("set_point", motor_output, queue_size=10)
    rospy.Subscriber("key", String , keyd_callback)
    rate = rospy.Rate(166)

    operationMode =  rospy.get_param("/op", 1)
    msg = motor_output()

    rospy.on_shutdown(shutdown)

    while not rospy.is_shutdown():
        duty_cicle = 0
        direction = ""
        aP = rospy.get_param("/vel", 10)

        if (operationMode == 1):
            rawPWM = aP * np.sin(rospy.get_time())
            duty_cicle = rawPWM
            direction = "forward" if rawPWM >= 0 else "backward"

        elif (operationMode == 2):
            period = rospy.get_param("/period",2)
            #duty_cicle = 255 if np.abs(np.sin(np.pi / period * rospy.get_time())) < np.sin(np.pi / 2 * aP) else -255
            if(np.sin(np.pi/period * rospy.get_time()) > 0):
                duty_cicle = aP
                direction = "forward"
            else:
                duty_cicle = -aP
                direction = "backward"

            #direction = "forward"
        elif(operationMode == 3):
            duty_cicle = aP
            direction = "forward"

        elif(operationMode == 4):
            if(k == 'A'):
                duty_cicle = -aP
                direction = "backward"
            elif(k == 'D'):
                duty_cicle = aP
                direction = "forward"
            else:
                duty_cicle = 0
                direction = "forward"

        msg.vel = duty_cicle
        msg.direction = direction

        pub.publish(msg)

        rate.sleep()

