#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String, UInt8
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

global motor1
global motor2
global state_buttons
global motors_initialized
global last_state

motor1 = 0
motor2 = 0
motorBoth = 0
vehicle_mode = 0
motors_initialized = 0
last_mode = 0

#Subscriber callback for the Joystick Topic
def joystick_listener(data):
    global motor1
    global motor2
    global motorBoth
    global state_buttons
    global motors_initialized

    #rospy.loginfo("Button 8: " + str(data.buttons[8]) + " Button 9: " + str(data.buttons[9]) + " Button 10: " + str(data.buttons[10]) + " Button 11: " + str(data.buttons[11]) + " Button 27: " + str(data.buttons[27]) + " Button 28: " + str(data.buttons[28]) + " Button 29: " + str(data.buttons[29]))

    #Open and close buttons for Right arm
    if data.buttons[16] == 1:
        motor1 = 1
    elif data.buttons[17] == 1:
        motor1 = -1
    else:
        motor1 = 0
    
    #Open and close buttons for Left arm
    if data.buttons[15] == 1:
        motor2 = 1
    elif data.buttons[14] == 1:
        motor2 = -1
    else:
        motor2 = 0
    #Operate both arms
    if data.buttons[1] == 1:
	motorBoth = -1
    elif data.buttons[0] == 1:
	motorBoth = 1
    else:
	motorBoth = 0

#region Vehicle Mode
def toggle_vehicle_mode(toggle):
    global vehicle_mode
    vehicle_mode = toggle.data

#Main runtime for publishing commands. 
def epos_cmd():
    global motors_initialized
    global last_mode, vehicle_mode

    rospy.init_node('coala_arms_controller', anonymous=True)
    rospy.Subscriber('/acomar/joy', Joy, joystick_listener)
    rospy.Subscriber('/acomar/vehicle_mode', UInt8, toggle_vehicle_mode)
    motor1_pub = rospy.Publisher("/maxon/canopen_motor/base_link1_joint_velocity_controller/command", Float64, queue_size=1)
    motor2_pub = rospy.Publisher("/maxon/canopen_motor/base_link2_joint_velocity_controller/command", Float64, queue_size=1)
    motorStatus_pub = rospy.Publisher("/maxon/canopen_motor/status", String, queue_size=1)
    init_motors = rospy.ServiceProxy('/maxon/driver/init', Trigger)
    halt_motors = rospy.ServiceProxy('/maxon/driver/halt', Trigger)
    recover_motors = rospy.ServiceProxy('/maxon/driver/recover', Trigger)
    shutdown_motors = rospy.ServiceProxy('/maxon/driver/shutdown', Trigger)
    motorStatus_pub.publish("Disarmed")

    loop_rate = 50.0
    rate = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        if motor1 == 1:
           motor1_pub.publish(800)
          # rospy.loginfo("While loop button 1 - up")
        elif motor1 == -1:
           motor1_pub.publish(-800)
          # rospy.loginfo("While loop button 1 - down")
        else:
	   if motorBoth == 0:
		motor1_pub.publish(0)
          # rospy.loginfo("While loop button 1 - idle")

        if motor2 == 1:
            motor2_pub.publish(800)
           # rospy.loginfo("While loop button 2 - up")
        elif motor2 == -1:
            motor2_pub.publish(-800)
           # rospy.loginfo("While loop button 2 - down")
        else: 
	    if motorBoth == 0:
		motor2_pub.publish(0)
           # rospy.loginfo("While loop button 2 - idle")

        if motorBoth == -1:
	    motor1_pub.publish(800)
	    motor2_pub.publish(-800)
	elif motorBoth == 1:
	    motor1_pub.publish(-800)
	    motor2_pub.publish(800)
 
        if vehicle_mode == 4:
            if motors_initialized == 4:
                shutdown_motors()
                motorStatus_pub.publish("Turned_Off")

        if vehicle_mode == 3:
            if last_mode != 3:
                halt_motors()
		motorStatus_pub.publish("Disarmed")

        if vehicle_mode == 2:
            if last_mode != 2:
                if motors_initialized == 0:
                    motors_initialized = 1
                    init_motors()
		    motorStatus_pub.publish("Armed")
                else:
                    recover_motors()
		    motorStatus_pub.publish("Armed")

        if vehicle_mode == 1:
            if last_mode != 1 and last_mode != 0:
                halt_motors()
		motorStatus_pub.publish("Disarmed")

        last_mode = vehicle_mode

        rate.sleep()

if __name__ == '__main__':
    try:
        epos_cmd()
    except rospy.ROSInterruptException:
        pass
