# run the cell to record images
from unifr_api_epuck import wrapper
import os
import numpy as np
# create directory
try: 
    os.mkdir("./images") 
except OSError as error: 
    print(error)  

MY_IP = '192.168.2.202'
robot = wrapper.get_robot(MY_IP)

robot.init_camera('./images') # define your working directory for storing images (do not forget to create it)
robot.sleep(3)

stepcounter = 0
n = 25

while robot.go_on():
	
	if stepcounter % n == 0:
		robot.init_camera('./images')
		stepcounter += 1
	if stepcounter % n == 1:
		[r, g, b] = np.array(robot.get_camera())
		meanR = np.mean(r)
		meanG = np.mean(g)
		meanB = np.mean(b)	
		if meanR > 150 and meanG < 150 and meanB < 150:
			robot.enable_led(1, 100, 0, 0)
			robot.enable_led(3, 100, 0, 0)
			robot.enable_led(5, 100, 0, 0)
			robot.enable_led(7, 100, 0, 0)
		elif meanG > 100 and meanR < 120 and meanB < 120:
			robot.enable_led(1, 0, 95, 0) #setting the LED to 100 Green resulted in multiple replicated 									crashes, EPUCK 4202
			robot.enable_led(3, 0, 95, 0)
			robot.enable_led(5, 0, 95, 0)
			robot.enable_led(7, 0, 95, 0)
		elif meanB > 100 and meanR < 100 and meanG < 120	:
			robot.enable_led(1, 0, 0, 100)
			robot.enable_led(3, 0, 0, 100)
			robot.enable_led(5, 0, 0, 100)
			robot.enable_led(7, 0, 0, 100)
		else:
			robot.disable_all_led()
	
robot.clean_up()
