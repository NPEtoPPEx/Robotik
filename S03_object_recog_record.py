# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper
import numpy as np
import os

# create directory
try: 
    os.mkdir("./img") 
except OSError as error: 
    print(error) 

MY_IP = '192.168.2.202'
MAX_STEPS = 200

robot = wrapper.get_robot(MY_IP)

robot.initiate_model()
robot.init_camera("./img")

#open file for writing
data = open("object_recog.csv", "w")

if data == None:
    print('Error opening data file!\n')
    quit

#write header in CSV file
data.write('step,x_center,y_center,width,height,conf,label\n')


# wait 3 seconds before starting
robot.sleep(3)

robot.set_speed(-1.5,1.5)
step = 0

while robot.go_on() and step < MAX_STEPS:
    print("hi")
    img = np.array(robot.get_camera())
    detections = robot.get_detection(img,0)
    outlier = False
    #Just save the detection
    for item in detections:
        data.write("{},{},{},{},{},{},{}\n".format(step,item.x_center,item.y_center,item.width,item.height,item.confidence,item.label))
        if item.y_center < 50:
            outlier = True 

    if outlier:
        robot.save_detection()  

    step += 1
    

    
data.close()
robot.clean_up()
