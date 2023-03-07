# run this code to generate IR sensor data 
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.202' # change robot number

MAX_PROX = 120
NORM_SPEED = 1.5

robot = wrapper.get_robot(MY_IP)

robot.init_sensors()
robot.calibrate_prox()

# wait 1 seconds before starting
robot.sleep(1)

while robot.go_on():

#get prox values
	ps_values = robot.get_calibrate_prox()
	#we choose the weights 0.4 --> .3 --> .2 --> .1 (= 1) so we have a max value of 4095
	proxRight = (0.4* ps_values[0] + 0.3 *ps_values[1] + 0.3 *ps_values[2] + 0.2 *ps_values[3]) / 1.2
	proxLeft = (0.4* ps_values[7] + 0.3 *ps_values[6] + 0.3 *ps_values[5] + 0.2 *ps_values[4]) / 1.2
	print('left: ' + str(proxLeft) + ' right : ' + str(proxRight)) 
	
	if proxRight > MAX_PROX or	 proxLeft > MAX_PROX:
		dsL = (NORM_SPEED * proxLeft) / MAX_PROX
		dsR = (NORM_SPEED * proxRight) / MAX_PROX
		speedL = NORM_SPEED - dsL
		speedR = NORM_SPEED - dsR
		robot.set_speed(speedR, speedL)
	else:
		robot.set_speed(NORM_SPEED, NORM_SPEED)
	

	
robot.clean_up()

