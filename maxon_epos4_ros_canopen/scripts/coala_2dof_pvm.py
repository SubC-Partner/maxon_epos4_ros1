#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64 
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

global motor1
global motor2
global state_buttons
global motors_initialized
global last_state

motor1 = 0
motor2 = 0
state_buttons = 0
motors_initialized = 0
last_state = 0

#Subscriber callback for the Joystick Topic
def joystick_listener(data):
    global motor1
    global motor2
    global state_buttons
    global motors_initialized

    #rospy.loginfo("Button 8: " + str(data.buttons[8]) + " Button 9: " + str(data.buttons[9]) + " Button 10: " + str(data.buttons[10]) + " Button 11: " + str(data.buttons[11]) + " Button 27: " + str(data.buttons[27]) + " Button 28: " + str(data.buttons[28]) + " Button 29: " + str(data.buttons[29]))

    #Open and close buttons for Right arm
    if data.buttons[8] == 1:
        motor1 = 1
    elif data.buttons[9] == 1:
        motor1 = -1
    else:
        motor1 = 0
    
    #Open and close buttons for Left arm
    if data.buttons[10] == 1:
        motor2 = 1
    elif data.buttons[11] == 1:
        motor2 = -1
    else:
        motor2 = 0

    #State buttons on the joystick to init the motor
    if data.buttons[27] == 1:
        state_buttons = 0
    elif data.buttons[28] == 1:
        state_buttons = 1
    elif data.buttons[29] == 1:
        state_buttons = 2

#Main runtime for publishing commands. 
def epos_cmd():
    global motors_initialized
    global last_state

    rospy.init_node('coala_arms_controller', anonymous=True)
    rospy.Subscriber('/acomar/joy', Joy, joystick_listener)
    motor1_pub = rospy.Publisher("/maxon/canopen_motor/base_link1_joint_velocity_controller/command", Float64, queue_size=1)
    motor2_pub = rospy.Publisher("/maxon/canopen_motor/base_link2_joint_velocity_controller/command", Float64, queue_size=1)
    init_motors = rospy.ServiceProxy('/maxon/driver/init', Trigger)
    halt_motors = rospy.ServiceProxy('/maxon/driver/halt', Trigger)
    recover_motors = rospy.ServiceProxy('/maxon/driver/recover', Trigger)
    shutdown_motors = rospy.ServiceProxy('/maxon/driver/shutdown', Trigger)

    loop_rate = 50.0
    rate = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        if motor1 == 1:
           motor1_pub.publish(500)
           rospy.loginfo("While loop button 1 - up")
        elif motor1 == -1:
           motor1_pub.publish(-500)
           rospy.loginfo("While loop button 1 - down")
        else:
           motor1_pub.publish(0)
          # rospy.loginfo("While loop button 1 - idle")

        if motor2 == 1:
            motor2_pub.publish(500)
            rospy.loginfo("While loop button 2 - up")
        elif motor2 == -1:
            motor2_pub.publish(-500)
            rospy.loginfo("While loop button 2 - down")
        else:
            motor2_pub.publish(0)
           # rospy.loginfo("While loop button 2 - idle")


        #State machine for the button on the joystick
        #State 0 Red LED
        if state_buttons == 0:
            if motors_initialized == 1:
                motors_initialized = 0
                shutdown_motors()
        #State 1 Purple LED
        if state_buttons == 1:
            if last_state != 1:
                if last_state == 0:
                    pass
                elif last_state == 2:
                    halt_motors()
                    pass
        #State 2 Blue LED
        if state_buttons == 2:
            if last_state == 1:
                if motors_initialized == 0:
                    motors_initialized = 1
                    init_motors()
                else:
                    recover_motors()
        
        last_state = state_buttons

        rate.sleep()

if __name__ == '__main__':
    try:
        epos_cmd()
    except rospy.ROSInterruptException:
        pass
