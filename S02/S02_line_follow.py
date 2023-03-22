# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.202'
MAX_SPEED = 1.5;

robot = wrapper.get_robot(MY_IP)

robot.init_ground()
# wait 3 seconds before starting
robot.sleep(3)


while robot.go_on():
	left = robot.get_ground()[0]
	middle = robot.get_ground()[1]
	right = robot.get_ground()[2]
	
	if left <= 350 and middle <= 350 and right >900:
		robot.set_speed(MAX_SPEED)
		print("middle")
	elif left <= 350 and middle <= 350 and right <= 350:
		robot.set_speed(MAX_SPEED * 0.15, MAX_SPEED * (-0.15))
	elif left >= 350 and middle >= 350 and right <= 350:
		robot.set_speed(MAX_SPEED * 0.15, MAX_SPEED * (-0.15))
	elif left >= 350 and middle >= 350 and right >= 350:
		robot.set_speed(MAX_SPEED * (-0.15), MAX_SPEED * (0.15))
	
data.close()
robot.clean_up()
