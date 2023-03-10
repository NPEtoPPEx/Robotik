# Basic lover implementation
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.202'  # change robot number
robot = wrapper.get_robot(MY_IP)

NORM_SPEED = 1.5
MAX_PROX = 250
ps_values = None
proxRight = None
proxLeft = None
dsL = None
dsR = None
speedL = None
speedR = None
counter = 0

Lover = True

robot.init_sensors()
robot.calibrate_prox()


def getValues():

	global ps_values 
	global proxRight
	global proxLeft 
	global dsL  
	global dsR 
	global speedL 
	global speedR 
	ps_values = robot.get_calibrate_prox()
	proxRight = (0.4* ps_values[0] + 0.3 *ps_values[1] + 0.3 *ps_values[2] + 0.2 *ps_values[3]) / 1.2
	proxLeft = (0.4* ps_values[7] + 0.3 *ps_values[6] + 0.3 *ps_values[5] + 0.2 *ps_values[4]) / 1.2
	dsL = (NORM_SPEED * proxLeft) / MAX_PROX
	dsR = (NORM_SPEED * proxRight) / MAX_PROX
	speedL = NORM_SPEED - dsL
	speedR = NORM_SPEED - dsR


#infinite loop
while robot.go_on():
	
	if Lover:
		robot.enable_all_led()
		getValues()
		if speedL < 0.15 and speedR < 0.15:
			counter += 1	
			if counter > 50 :
				Lover = False
				counter = 0
				print(Lover)
		robot.set_speed(speedL, speedR)
	else:
		robot.disable_all_led()
		getValues()
		if proxRight < MAX_PROX and proxLeft < MAX_PROX:
			print(counter)
			counter += 1
			if counter > 200:	
    				Lover = True
    				counter = 0
    				print(Lover)
		robot.set_speed(speedR, speedL)
    	
robot.clean_up()
