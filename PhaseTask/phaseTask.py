from unifr_api_epuck import wrapper
import os # for log files
import numpy as np
import os
import signal
# create directory
try: 
    os.mkdir("./img") 
except OSError as error: 
    print(error) 

MY_IP = '192.168.2.130'
robot = wrapper.get_robot(MY_IP)
robot.initiate_model()
robot.init_camera("./img")
def handler(signum, frame):
    robot.clean_up()

signal.signal(signal.SIGINT, handler)
#open file for writing
data = open("object_recog.csv", "w")
if data == None:
    print('Error opening data file!\n')
    quit
#write header in CSV file
data.write('step,x_center,width,label\n')
# wait 3 seconds before starting
robot.sleep(3)
robot.init_sensors()
robot.calibrate_prox()

step = 0
stepcounter = 0
n = 10
MAX_STEPS = 600		
MAX_SPEED = 0.4
MAX_PROX = 100
turningSteps = 1450	# 1300 was 2 turns
greenBlockCenter = 0
centerX = 80
phase = 0	# 0 = Turn 360 , 1 Search Green Block , 2 Center Green Block 
turn = False
PID_MAX_DS = 0.5
PID_WALL_TARGET = 100
a = 4
b = 2
c = 1
d = 0

# PID parameters
K = 0.005
T_D = 0.0306
T_I = 999999


class PID:

    TIME_STEP = 64

    def __init__(self, k, t_i, t_d):
        self.error = 0
        self.deriv = 0
        self.integ = 0
        self.K = k
        self.T_I = t_i
        self.T_D = t_d

    def compute(self,prox,target):    
        prev_err = self.error
        self.error = prox - target

        self.deriv = (self.error - prev_err)*1000/self.TIME_STEP
        self.integ += self.error*self.TIME_STEP/1000

        #return self.K * ( self.error + 1.0 / self.T_I * self.integ + self.T_D * self.deriv)
        return self.P() + self.I() + self.D()

    def P(self) :
        return self.K * self.error    

    def I(self) :
        return self.K * (self.integ/self.T_I)    

    def D(self) :
        return self.K * (self.T_D * self.deriv)    
    


pid = PID( K, T_I, T_D)
robot.set_speed(-MAX_SPEED,MAX_SPEED)


while robot.go_on():
	print(phase)
	robot.enable_led(0)
	if phase == 0: # turn a full rotation	
		if stepcounter % n == 0:
			robot.init_camera()
		if stepcounter % n == 1:
			img = np.array(robot.get_camera())
			detections = robot.get_detection(img,0.9)
			for item in detections:
				if item.label == "Red Block":
					robot.enable_led(7, 100, 0 ,0)
				elif item.label == "Green Block":
					robot.enable_led(7, 0, 95, 0)
				elif item.label == "Blue Block":
					robot.enable_led(7, 0, 0, 100)
				elif item.label == "Black Block":
					robot.enable_led(7, 100, 95, 100)
				else:
					robot.disable_led(7)
		step += 1
		stepcounter += 1
		
		if step >= MAX_STEPS:
			phase = 1
	elif phase == 1: #center towards green block
		if stepcounter % n == 0:
			robot.init_camera()
		if stepcounter % n == 1:
			img = np.array(robot.get_camera())
			detections = robot.get_detection(img,0.9)
			for item in detections:
				data.write("{},{},{},{}\n".format(step,item.x_center,item.width,item.label))
				if item.label == "Red Block":
					robot.enable_led(7, 100, 0 ,0)
				elif item.label == "Blue Block":
					robot.enable_led(7, 0, 0, 100)
				elif item.label == "Black Block":
					robot.enable_led(7, 100, 95, 100)
				elif item.label == "Green Block":
					robot.disable_all_led()
					robot.enable_led(4, 100)
					greenBlockCenter = item.x_center
					deltaOffset = greenBlockCenter - centerX
					#if(abs(centerX - greenBlockCenter) < 20):
					robot.enable_led(7, 0, 100, 0)
					phase = 2
				else:
					robot.disable_led(7)
		stepcounter += 1		
	elif phase == 2:
			img = np.array(robot.get_camera())
			detections = robot.get_detection(img,0.9)
			for item in detections:
				if item.label == "Green Block":
					greenBlockCenter = item.	x_center
			print("Camera Center: " + str(centerX) + "Block Center :" + str(greenBlockCenter)) 
			deltaOffset = greenBlockCenter - centerX
			deltaSpeed = (MAX_SPEED - deltaOffset)/120
			robot.set_speed(-deltaSpeed, deltaSpeed)
			if(abs(centerX - greenBlockCenter) < 3):
				robot.disable_camera
				phase = 3
	elif phase == 3:
			ps_values = robot.get_calibrate_prox()
			#proxRight = (1* ps_values[0] + 0*ps_values[1] + 0 *ps_values[2] + 0*ps_values[3]) / 1
			#proxLeft = (1* ps_values[7] + 0*ps_values[6] + 0 *ps_values[5] + 0*ps_values[4]) / 1
			#dsL = (MAX_SPEED * proxLeft) / MAX_PROX
			#dsR = (MAX_SPEED * proxRight) / MAX_PROX
			#speedL = MAX_SPEED - dsL
			#speedR = MAX_SPEED - dsR
			robot.set_speed(MAX_SPEED, MAX_SPEED)
			if(ps_values[0] > 100 and ps_values[7] > 100):
				phase = 4
				step = 0;
	elif phase == 4:   
		if step < turningSteps:
			step += 1
			robot.set_speed(-MAX_SPEED, MAX_SPEED)
		else:
			phase = 5
	elif phase == 5: 	
	
			ps = robot.get_calibrate_prox()
			proxR = (a * ps[0] + b * ps[1] + c * ps[2] + d * ps[3]) / (a+b+c+d);
                      
    # compute PID response according to IR sensor value
			ds = pid.compute(proxR,PID_WALL_TARGET);      
    # make the robot turn towards the wall by default    
			ds += .05
			speedR = MAX_SPEED + ds
			speedL = MAX_SPEED - ds
			if (ps[0] > 150 and ps[7] > 150):	
				phase = 6
				step = 0
			robot.set_speed(speedL,speedR)
	elif phase == 6:
			if step < 250 :
				robot.set_speed(-MAX_SPEED, MAX_SPEED)
				step += 1	
			else:
				phase = 7
	elif phase == 7:
			ps = robot.get_calibrate_prox()
			proxL = (a * ps[7] + b * ps[6] + c * ps[5] + d * ps[4]) / (a+b+c+d);
                      
    # compute PID response according to IR sensor value
			ds = pid.compute(proxL,PID_WALL_TARGET);      
    # make the robot turn towards the wall by default    
			ds += .05
			speedR = MAX_SPEED - ds
			speedL = MAX_SPEED + ds
			if (ps[0] > 100 and ps[7] > 100):	
				phase = 8
			robot.set_speed(speedL,speedR)
	elif phase == 8:
			robot.set_speed(-MAX_SPEED, MAX_SPEED)
			print(robot.get_microphones())
		
		
data.close()
robot.clean_up()
