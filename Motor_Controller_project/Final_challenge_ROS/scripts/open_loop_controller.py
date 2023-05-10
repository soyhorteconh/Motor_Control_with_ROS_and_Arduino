#!/usr/bin/env python
import rospy
import csv
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from final_challenge.msg import motor_msg
from final_challenge.msg import motor_output
import numpy as np

# Controller variables
error = 0.0
error_int = 0.0
error_d = 0.0
error_prev = 0.0

prevTime = 0

# Kp, Ki and Kd values
kp = rospy.get_param("/control_kp",12)
ki = rospy.get_param("/control_ki",50)
kd = rospy.get_param("/control_kd",0.421)
output = 0

# Callback variables
receiveSetPoint_vel = 0
receiveSetPoint_direction = ""
receiveMotorOutput_direction = ""
receiveMotorOutput_vel = 0
flag = 0

motor_input_msg = motor_msg()

# Callback setpoint function
def set_point_callback(msg):
    global receiveSetPoint_vel, receiveSetPoint_direction
    receiveSetPoint_vel = msg.vel
    receiveSetPoint_direction = msg.direction

# Callback motor output function
def motor_output_callback(msg):
    global receiveMotorOutput_vel, receiveMotorOutout_direction, flag
    receiveMotorOutput_vel = msg.vel
    receiveMotorOutput_direction = msg.direction
    flag = msg.flag

def stop():
    motor_input_msg.pwm = 0
    motor_input_pub.publish(motor_input_msg)


# PID controller
def PID(setPoint, motorOutput):
    global error, error_prev, error_d, error_int, ki, kp, kd, motor_input_msg, prevTime, flag, output
    currentTime = rospy.get_time()
    dt = currentTime - prevTime

    # Error
    error = setPoint - motorOutput

    # Integration error
    if(flag == 1):
        error_int += error * dt

    # Derivation Error
    error_d = (error - error_prev) / dt

    # Controller output
    output = kp*error + ki*error_int + kd*error_d
    output = output/255
    #output = setPoint

    # File generation
    #row = [(currentTime), (output), (motorOutput)]
    #writer.writerow(row)

    # Motor
    if(output > 1):
        output = 1
    elif(output < -1):
        output = -1

    if(flag == 0):
        output = 0

    motor_input_msg.pwm =np.abs(output)
    if output >= 0:
        motor_input_msg.direction = "forward"
    else:
        motor_input_msg.direction = "backward"

    error_prev = error
    prevTime = currentTime

if __name__ == '__main__':
    #f = open("/home/andrea/Documents/vel.csv", 'w')
    #writer = csv.writer(f)

    # Node
    rospy.init_node("open_loop_controller")
    # Publishers
    motor_input_pub = rospy.Publisher("motor_input", motor_msg, queue_size=10)
    error_pub = rospy.Publisher("error", Float32, queue_size = 10)
    prevTime = rospy.get_time()
    # Subscribers
    rospy.Subscriber("set_point", motor_output, set_point_callback)
    rospy.Subscriber("motor_output", motor_output, motor_output_callback)

    rate = rospy.Rate(166)
    rospy.on_shutdown(stop)

    while not rospy.is_shutdown():

        PID(receiveSetPoint_vel, receiveMotorOutput_vel)
        motor_input_pub.publish(motor_input_msg)

        rate.sleep()