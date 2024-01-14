from whisper_mic_igor import WhisperMicIgor #requires a conda env with whisper-mic installed and opencv (opencv-python) and ros (rospkg) 
import time
import re
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

from ctypes import *

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
  #print('error received')
  pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
# Set error handler of asound libray to the previous one
asound.snd_lib_error_set_handler(c_error_handler)

# create mic module
mic = WhisperMicIgor(model="base",english=True,verbose=False,energy=900,pause=1.8,dynamic_energy=False,save_file=False, model_root="~/.cache/whisper",mic_index=None) #in the console whisper_mic --help for info on usage

#---------------------------------------------------------

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
    #model = torch.hub.load('/home/colors/yolov5', 'custom', path='/home/colors/yolov5/best.pt', source='local')    # or yolov5n - yolov5x6 or custom
    model = torch.hub.load('/home/colors/yolov5', 'custom', path='/home/colors/yolov5/beng_train_objects/best.pt', source='local')  
    model.conf = 0.40  # confidence threshold (0-1)
    model.iou = 0.40
    # or yolov5n - yolov5x6 or custom
    

    pipe = rs.pipeline()
    cfg  = rs.config()
    time.sleep(1)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    pipe.start(cfg)


    time.sleep(1)
    frame = pipe.wait_for_frames()
    time.sleep(1)
    color_frame = frame.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())
    results = model(color_image)

    print(results.pandas().xyxy[0])
    #time.sleep(2)
    #cv2.imshow('rgb', color_image)
    #cv2.waitKey(0)  # Wait for any key press
    #cv2.destroyAllWindows()  # Close all windows
    time.sleep(2)
    pipe.stop()
    return results

def get_object(result_line):
    x0,y0,x1,y1,confi,cla = result_line
    x_avg=(x0+x1)/2
    y_avg=(y0+y1)/2
    return(x_avg,y_avg)

#---------------------------------------------------
def get_direction(baxter_coord, key):
    baxter_coord2 = [0,0] 
    if(key == "right"):
        baxter_coord2[1] = baxter_coord[1] - 0.08 
        baxter_coord2[0] = baxter_coord[0]
    elif(key =="left"):
        baxter_coord2[1] = baxter_coord[1] + 0.08 
        baxter_coord2[0] = baxter_coord[0]
    elif(key == "above"):
        baxter_coord2[0] = baxter_coord[0] + 0.08 
        baxter_coord2[1] = baxter_coord[1]
    elif(key== "below"):
        baxter_coord2[0] = baxter_coord[0] - 0.08 
        baxter_coord2[1] = baxter_coord[1]
   # elif(key== "inside" or key == "in" or key == "into" or key == "on"):
   #     baxter_coord2[2] = baxter_coord[2] + 0.02 
    else:  
        baxter_coord2 = baxter_coord

    return baxter_coord2

# code execution
def put_action(baxter_coord, isInside=False):
    print("This is put action.")

    robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],p.z],[1,0,0,0])  # go to given location  
    if(not isInside):
        robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],-0.13897744],[1,0,0,0])  # go down in that location (change it later)
    else:
        robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],-0.09897744],[1,0,0,0])
    #grip_left.open() #drop the opject
    robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],p.z],[1,0,0,0]) 
    robot.move_to_neutral()
  
def hold_action(baxter_coord):
    
    print("This is the hold action.")

    robot.move_to_neutral()
    print("This is the hold action.2")
  #  robot.set_cartesian_position([0.745888,0.36672261,-0.065772],[1,0,0,0])
 #   grip_left.open()
    print("This is the hold action.3")
    robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],p.z],[1,0,0,0])
   # robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],-0.13897744],[1,0,0,0])
    print(baxter_coord[0],baxter_coord[1])
  #  grip_left.close()
    robot.set_cartesian_position([baxter_coord[0],baxter_coord[1],p.z],[1,0,0,0])
  #  robot.move_to_neutral()

    #msg = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
    #p = msg.pose.position
    #print(p)

#def take_action(obj):
#    print("This is the take action.")
#    print("take {}".format(obj))

#---------------ROBOT INITIALIZING -----------------

rospy.init_node("testing")
rospy.sleep(2.0)
robot = baxter.BaxterRobot(rate=100, arm="left")
rospy.sleep(2.0)
robot.set_robot_state(True)

grip_left = Gripper('left', CHECK_VERSION)
grip_left.calibrate()
if grip_left.error():
    grip_left.reset()
if (not grip_left.calibrated() and
    grip_left.type() != 'custom'):
    grip_left.calibrate()
msg = rospy.wait_for_message("/robot/limb/left/endpoint_state", EndpointState)
p = msg.pose.position
print(p)
q = msg.pose.orientation

#----Whisper --------

commands = ["collect", "hold", "take", "exit", "quit","put", "give"]
directions = ["under", "on", "right", "left", "above", "below", "in front of", "behind", "apart", "together", "next to", "inside"]
articles = ["a", "an", "the"]
time.sleep(2.0)
key = "right" 

flag = 1
while(flag):
    #if keyboard.is_pressed(key):
    print("Give a command")
    result = mic.listen(phrase_time_limit=20)
    #result = input("ENTER YOUR INPUT NOW: ") # TODO change later to mic
    #result = "Hold red lego" #for test only, remove this later
    print("END")
    print("result:", result)
    
    if("red lego" in result.lower()):
        result = result.replace("red lego", "redlego")

    if("yellow lego" in result.lower()):
        result = result.replace("yellow lego", "yellowlego")

    if("blue lego" in result.lower()):
        result = result.replace("blue lego", "bluelego")    

    if("white lego" in result.lower()):
        result = result.replace("white lego", "whitelego")  
    if("containers" in result.lower()):
        result = result.replace("containers", "container")  

    sentences = re.split('.,', result)

    command_counter=1
    for sentence in sentences:
        sentence.strip()
        print("Sentence{}:".format(command_counter), sentence)

        words = sentence.split(" ")
        filtered_words = [word for word in words if word.lower() != "the"]
        result_sentence = " ".join(filtered_words)
        words = result_sentence.split(" ")

        obj=""
        command=''
        direction= ""
        direction_obj = ""
     
        for i in range (len(words)):
            word = words[i].lower()
            if(i < len(words)-1):
                next_word = words[i+1].lower()
            if word in articles:  #remove articles
                continue
            elif word in commands:
                command = word
                obj = next_word
            elif word in directions:
                direction = word
                if(next_word):
                    direction_obj = next_word
    
        


        #ask yolo the obj
        time.sleep(5)
        if(obj != '' and command!=''):
            if (command == "exit" or command == "quit"):
                print("Exiting...")
                flag = 0
                break

            #Get Position
            results=get_position()
            print("results:", results)
            detected_objects_df = results.pandas().xyxy[0]
            print("detected_objects_df:\n" ,detected_objects_df)
            
            object_names = detected_objects_df['name']
            print("object_names:\n",object_names)
            print("len: ",len(object_names))

            baxter_coord = [0,0]
            baxter_coord2 = [0,0]

            index = 0
            for i in range(len(object_names)):
                object_name= object_names[i]
                if object_name.strip() == obj.strip():
                    print("~Obj found: ", object_name.strip())
                    index = i
                    #print("results.xyxy[0][{}].cpu().numpy():\n".format(i), results.xyxy[0][i].cpu().numpy())
                    my_tuple=get_object(results.xyxy[0][i].cpu().numpy())

                    # my_tuple=get_object(results.xyxy[0][0].cpu().numpy())
                    # Assuming results.pandas().xyxy[0] is your DataFrame
                    
                    #print("my_tuple:",my_tuple)
                    # print(my_tuple['name'])
                    baxter_coord=convert(my_tuple)
                    #print("baxter_coord:",baxter_coord)

                elif(object_name.strip() == direction_obj.strip()):
                    print("~Obj found: ", object_name.strip())
                    #print("results.xyxy[0][{}].cpu().numpy():\n".format(i), results.xyxy[0][i].cpu().numpy())
                    my_tuple=get_object(results.xyxy[0][i].cpu().numpy())
                    baxter_coord2=convert(my_tuple)

            if (command == "hold" or command == "grab"):

                hold_action(baxter_coord)
            
            elif(command == "put"):
                hold_action(baxter_coord)
                baxter_coord3 = get_direction(baxter_coord2, direction)
                if(direction == "inside" or direction == "in" or direction == "into" or direction == "on"):
                    put_action(baxter_coord3, True)
                else:
                    put_action(baxter_coord3)
                
           
        robot.move_to_neutral()
        exit()


    #filtered_words = [word for word in words if word.lower() != "the"]
    #example: Hold the apple. --> Hold apple.

    #    if (obj):
    #        if (command == "hold" or command == "grab"):
    #            hold_action(obj)
    #        elif (command == "take"):
    #            take_action(obj)
    #    command_counter +=1
        


    #wait for action