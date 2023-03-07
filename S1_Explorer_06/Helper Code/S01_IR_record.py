# run this code to generate IR sensor data 
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.202' # change robot number
MAX_STEPS = 300	

robot = wrapper.get_robot(MY_IP)

robot.init_sensors()
robot.calibrate_prox()

#open file for writing
data = open("IRsensors.csv", "w")

if data == None:
    print('Error opening data file!\n')
    quit

#write header in CSV file
data.write('step,')
for i in range(1):
    data.write('Average PS'+',')
data.write('\n')

# wait 5 seconds before starting
robot.sleep(5)

for step in range(MAX_STEPS):
    robot.go_on()
    robot.set_speed(-1.5,-1.5)
    ps_values = robot.get_calibrate_prox()
    PROX_RIGHT_FRONT = ps_values[0]
    PROX_LEFT_FRONT = ps_values[7]
    PROX_AVG_Values = (PROX_RIGHT_FRONT + PROX_LEFT_FRONT) / 2
    #ps = robot.get_prox() # uncomment if analyzing uncalibrated sensor data

    #write a line of data 
    data.write(str(step)+',')
    data.write(str(PROX_AVG_Values)+',')
    data.write('\n')
    
data.close()

robot.clean_up()

