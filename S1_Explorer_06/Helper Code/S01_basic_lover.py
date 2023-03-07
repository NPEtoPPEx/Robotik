# Basic lover implementation
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.202'  # change robot number
robot = wrapper.get_robot(MY_IP)

NORM_SPEED = 1.5
MAX_PROX = 250

robot.init_sensors()
robot.calibrate_prox()

#infinite loop
while robot.go_on():
	ps_values = robot.get_calibrate_prox()
	proxRight = (0.4* ps_values[0] + 0.3 *ps_values[1] + 0.3 *ps_values[2] + 0.2 *ps_values[3]) / 1.2
	proxLeft = (0.4* ps_values[7] + 0.3 *ps_values[6] + 0.3 *ps_values[5] + 0.2 *ps_values[4]) / 1.2
	dsL = (NORM_SPEED * proxLeft) / MAX_PROX
	dsR = (NORM_SPEED * proxRight) / MAX_PROX
	speedL = NORM_SPEED - dsL
	speedR = NORM_SPEED - dsR
	robot.set_speed(speedL, speedR)
    
robot.clean_up()
