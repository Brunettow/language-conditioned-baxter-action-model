#!/usr/bin/env python3
import rospy
import baxter
import numpy as np
from baxter_core_msgs.msg import EndpointState
from pynput.keyboard import Key, Listener
from baxter_interface import Gripper
from baxter_interface import CHECK_VERSION



def to_control(key):
    delta = 0.03
    if key == "w":
        delta_position[0]=delta
    
    if key== "s":
        delta_position[0]=-delta
        
    if key == "a":
        delta_position[1]=delta

    if key== "d":
        delta_position[1]=-delta        

    if key == "k":
        delta_position[2]=delta  

    if key== "l":
        delta_position[2]=-delta
        
    #gripper
    if key== "m":
        grip_right.open()

    if key== "n":
        grip_right.close()


def on_press(key):
    print()
    msg = rospy.wait_for_message("/robot/limb/right/endpoint_state", EndpointState)
    p = msg.pose.position
    print(p)
    q = msg.pose.orientation
    to_control(key.char)
    robot.get_cartesian_position([p.x+delta_position[0], p.y+delta_position[1], p.z+delta_position[2]], [1,0,0,0])
    
    print(delta_position)
    #delta_position[0]=0.0
    #delta_position[1]=0.0
    #delta_position[2]=0.0

def on_release(key):
    if key.char == ('q'):
        return False

    delta_position[0]=0.0
    delta_position[1]=0.0
    delta_position[2]=0.0


    if key.char == ('w'):
    	delta_position[0]=0.0


rospy.init_node("testing")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm="right")
rospy.sleep(2.0)
robot.set_robot_state(True)
grip_right = Gripper('right', CHECK_VERSION)
if grip_right.error():
    grip_right.reset()
if (not grip_right.calibrated() and
    grip_right.type() != 'custom'):
    grip_right.calibrate()

delta_position=[0.0,0.0,0.0]
delta_orientation=[0.0,0.0,0.0,0.0]
robot.move_to_neutral()

with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

