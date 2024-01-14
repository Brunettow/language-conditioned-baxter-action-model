import torch
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import pandas as pd
import rospy
import baxter
import numpy as np
from baxter_core_msgs.msg import EndpointState
from pynput.keyboard import Key, Listener
from baxter_interface import Gripper
from baxter_interface import CHECK_VERSION


def convert(tuple):
    #ust sol, ust sag, alt sol, alt sag
    cam_coordinates = [(38,11), (33,631), (249,11), (248, 631)] #do not forget x and y are replaced
    #robot_coordinates = [(0.91, -0.37), (0.67, 0.35), (0.95, -0.40), (0.67, 0.25)]

    #robot-sag, robot-sol, orta-sag, orta-sol
    robot_coordinates = [(0.66, -0.39), (0.66, 0.38), (0.95, -0.40), (0.95, 0.36)]

    x_c2 = (cam_coordinates[0][0] + cam_coordinates[1][0]) / 2
    x_c1 = (cam_coordinates[2][0] + cam_coordinates[3][0]) / 2
    x_r2 = (robot_coordinates[0][0] + robot_coordinates[1][0]) / 2
    x_r1 = (robot_coordinates[2][0] + robot_coordinates[3][0]) / 2

    y_c2 = (cam_coordinates[0][1] + cam_coordinates[2][1]) / 2
    y_c1 = (cam_coordinates[1][1] + cam_coordinates[3][1]) / 2
    y_r2 = (robot_coordinates[0][1] + robot_coordinates[2][1]) / 2
    y_r1 = (robot_coordinates[1][1] + robot_coordinates[3][1]) / 2

    x_replaced = tuple[1]
    y_replaced = tuple[0]

    x_new = x_r1 - (((x_r1 - x_r2) * (x_c1 - x_replaced)) / (x_c1 - x_c2))
    y_new = y_r1 - (((y_r1 - y_r2) * (y_c1 - y_replaced)) / (y_c1 - y_c2))

    return (x_new, y_new)


def get_position():
    #model = torch.hub.load('ultralytics/yolov5', 'custom', path='path/to/best.pt')  # local model
    model = torch.hub.load('/home/colors/yolov5', 'custom', path='/home/colors/yolov5/runs/train/exp6/weights/best.pt', source='local')  # local repo
    


    pipe = rs.pipeline()
    cfg  = rs.config()

    cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)

    pipe.start(cfg)

    frame = pipe.wait_for_frames()
    time.sleep(5)

    color_frame = frame.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())
    results = model(color_image)

    print(results.pandas().xyxy[0])
    time.sleep(10)    
    cv2.imshow('rgb', color_image)
    time.sleep(10)
    pipe.stop()
    return results,color_image

def get_avg(result_line):
    x0,y0,x1,y1,confi,cla = result_line
    x_avg=(x0+x1)/2
    y_avg=(y0+y1)/2
    return(x_avg,y_avg)

def get_color(rgb):
    red=[30, 80, 82]
    blue=[34, 29,  0]
    white=[35, 89, 86]
    if ((abs(rgb[0]-red[0])<10) and (abs(rgb[1]-red[1])<10) and (abs(rgb[2]-red[2])<10)):   
        return 'r'
    elif((abs(rgb[0]-blue[0])<10) and (abs(rgb[1]-blue[1])<10) and (abs(rgb[2]-blue[2])<10)):
        return 'b'
    else:
        return 'w'


#object_coordinates = (438,198) 
#new_robot_coordinates = convert(object_coordinates)

def get_object_at_position(objects, direction):
    item = objects[1]
    return item
        


        
    

results, color_img=get_position()
print(color_img)
print("results.xyxy[0][1].numpy()", results.xyxy[0][1].numpy())
print(type(results.xyxy))
object_list=[]
for i in range(len(results.xyxy[0])):
    my_tuple0=get_avg(results.xyxy[0][i].numpy())
    print(color_img[round(my_tuple0[0])][round(my_tuple0[1])])
    print(get_color(color_img[round(my_tuple0[0])][round(my_tuple0[1])]))
    object_list.append(my_tuple0)
    baxter_coord=convert(my_tuple0)
    print("baxter_coord {}: ".format(i), baxter_coord)

print("color_img[0][0]: ",color_img[0][0])






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
msg = rospy.wait_for_message("/robot/limb/right/endpoint_state", EndpointState)
p = msg.pose.position
print(p)
q = msg.pose.orientation


robot.move_to_neutral()
robot.set_cartesian_position([0.745888,0.36672261,-0.065772],[1,0,0,0])
time.sleep(2)
grip_right.open()
robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],p.z],[1,0,0,0])
time.sleep(2)
robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],-0.14897744],[1,0,0,0])
grip_right.close()

robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],p.z],[1,0,0,0])

msg = rospy.wait_for_message("/robot/limb/right/endpoint_state", EndpointState)
p = msg.pose.position
print(p)

