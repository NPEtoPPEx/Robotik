# run the cell to record images
from unifr_api_epuck import wrapper
import os

# create directory
try: 
    os.mkdir("./images") 
except OSError as error: 
    print(error)  

MY_IP = '192.168.2.202'
robot = wrapper.get_robot(MY_IP)

N_SAMPLES = 10

robot.init_camera('./images') # define your working directory for storing images (do not forget to create it)

#wait 3 seconds

robot.enable_led(1,100,100,100)
robot.enable_led(3,100,100,100)
robot.enable_led(5,100,100,100)
robot.enable_led(7,100,100,100)
robot.sleep(5)

step = 0
while robot.go_on() and step < N_SAMPLES :

    robot.take_picture('image'+str(step).zfill(3))
    step += 1

robot.clean_up()
