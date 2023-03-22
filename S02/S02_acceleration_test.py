# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.202'
MAX_SPEED = 0.05
step = 99
robot = wrapper.get_robot(MY_IP)

robot.init_ground()

#open file for writing
data = open("Acceleration.csv", "w")

if data == None:
    print('Error opening data file!\n')
    quit

#write header in CSV file
data.write('Acceleration at Max Speed:  ')
data.write('\n')
# wait 3 seconds before starting
robot.sleep(3)
for i in range(9):
	robot.set_speed(MAX_SPEED)
	for j in range(step):
		robot.go_on()
		data.write(str(j)+','+str(robot.get_acceleration()))
		data.write('\n')
	robot.set_speed(0)
	robot.sleep(1)
	MAX_SPEED = MAX_SPEED * 2;

data.close()

robot.clean_up()
