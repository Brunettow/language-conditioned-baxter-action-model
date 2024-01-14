#!/usr/bin/env python3
import rospy
import baxter
import numpy as np
from baxter_core_msgs.msg import EndpointState
from pynput.keyboard import Key, Listener


def to_control(key,p,q):
    print(p)
    delta = 0.1
    if key == "w":
        if p[0]<0.9:
            p[0]+=delta
            robot.set_cartesian_position(p, q)
            print('go +x')
    
    if key== "s":
        if p[0]>-0.9:
            p[0]-=delta
            robot.set_cartesian_position(p, q)
            print('go +x')
        
    if key == "d":
        if p[1]<0.9:
            p[1]+=delta
            robot.set_cartesian_position(p, q)
            print('go +y')
    
    if key== "a":
        if p[1]>-0.9:
            p[1]-=delta
            robot.set_cartesian_position(p, q)
            print("go -y")
        
    if key == "k":
        if p[2]<0.9:
            p[2]+=delta
            robot.set_cartesian_position(p, q)

            print('go +z')
    
    if key== "l":
        if p[2]>-0.9:
            p[2]-=delta
            robot.set_cartesian_position(p, q)
            print("go -z")
        
    

def on_press(key):
    msg = rospy.wait_for_message("/robot/limb/right/endpoint_state", EndpointState)
    position = msg.pose.position
    orientation = msg.pose.orientation
    p=[position.x,position.y,position.z]
    q=[orientation.x,orientation.y,orientation.z,orientation.w]
    to_control(key.char, p, q)

def on_release(key):
    if key.char == ('q'):
        # Stop listener
        return False

# Collect events until released

rospy.init_node("testing")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm="right")
rospy.sleep(2.0)
robot.set_robot_state(True)
msg = rospy.wait_for_message("/robot/limb/right/endpoint_state", EndpointState)
position = msg.pose.position
orientation = msg.pose.orientation
p=[position.x,position.y,position.z]
q=[orientation.x,orientation.y,orientation.z,orientation.w]
print(p)
print(type(p))
robot.move_to_neutral()
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

