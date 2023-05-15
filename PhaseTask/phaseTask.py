#imports
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

#initiate robot
MY_IP = '192.168.2.187'
robot = wrapper.get_robot(MY_IP)
robot.initiate_model()

def handler(signum, frame):
    robot.clean_up()
signal.signal(signal.SIGINT, handler)



#open file for writing
data = open("phaseTask.csv", "w")
if data == None:
    print('Error opening data file!\n')
    quit
#write header in CSV file
data.write("Phase 1 Object Recognition\n")
data.write('step,x_center,width,label\n')



#initiate senors
robot.sleep(3)
robot.init_sensors()
robot.calibrate_prox()
robot.init_camera("./img")

#attributes
#camera counters
n = 10 
stepcounter = 0

#object position attributes
greenBlockCenter = 0
centerX = 80

#Behaviour attributes
MAX_SPEED = 0.4
MAX_PROX = 100

step = 0    # Step counter
phase = 0	# Keeps Track which phase we are

stepTemp = 0 #ugly Temp counter for CSV


#PID Attributes
PID_MAX_DS = 0.5
PID_WALL_TARGET = MAX_PROX
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


#Defines Robot Behaviour
while robot.go_on():
	
	if phase == 0: # Phase 0 (1) Turn 360°, take pictures and enable corresponding LED to matching Color
		#enable correct Phase LED
		robot.enable_led(0)
		robot.set_speed(-MAX_SPEED,MAX_SPEED) #rotate on itself	
		
		if stepcounter % n == 0: #helps avoid crashing
			robot.init_camera()
		if stepcounter % n == 1:
			img = np.array(robot.get_camera())
			detections = robot.get_detection(img,0.9) #use implemented object recognition API and change LED Color depending which object is detected
			for item in detections:
				data.write("{},{},{},{}\n".format(step, item.x_center, item.width, item.label))
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
		#increase counters
		step += 1
		stepcounter += 1
		
		if step >= 600: #did a 360° turn and now switches to phase 1
			phase = 1
			step = 0

	elif phase == 1: #Phase 1 (1) continues turning, until it sees a Green Block, giving the Illusion of doing 1 and 3/4 turns
	
		if stepcounter % n == 0: #helps avoid crashing
			robot.init_camera()
		if stepcounter % n == 1:
			
			img = np.array(robot.get_camera())
			detections = robot.get_detection(img,0.9)  #use implemented object recognition API and change LED Color depending which object is detected
			
			for item in detections:
				data.write("{},{},{},{}\n".format(step, item.x_center, item.width, item.label))
				if item.label == "Red Block":
					robot.enable_led(7, 100, 0 ,0)
				elif item.label == "Blue Block":
					robot.enable_led(7, 0, 0, 100)
				elif item.label == "Black Block":
					robot.enable_led(7, 100, 95, 100)
				elif item.label == "Green Block":
					#calculate Object center and the corresponding Offset to Camera center
					greenBlockCenter = item.x_center		
					deltaOffset = greenBlockCenter - centerX
					
					robot.disable_all_led()
					robot.enable_led(4)
					robot.enable_led(7, 0, 100, 0)#Since Green has been detected, we turn on the LED
					
					phase = 2 #Since a green Block has been detected we go to the next phase
					step = 0
				else:
					robot.disable_led(7)
		#increase counters
		step += 1
		stepcounter += 1		
	
	elif phase == 2: #Phase 2 (1) Do slight center correction to be more robust when moving forward to the Green Block
	
			img = np.array(robot.get_camera())
			detections = robot.get_detection(img,0.9)
			
			for item in detections:
				if item.label == "Green Block":
					greenBlockCenter = item.x_center
			#Calculate the Offset and needed Speed to correct
			deltaOffset = greenBlockCenter - centerX
			deltaSpeed = (MAX_SPEED - deltaOffset)/120
			
			robot.set_speed(-deltaSpeed, deltaSpeed)
			#If the difference between the Green Block Center and Camera is < 3 Pixels, we consider it centered and move to the next phase
			if(abs(centerX - greenBlockCenter) < 3): 
				phase = 3
				step = 0
				robot.set_speed(MAX_SPEED, MAX_SPEED) #sets the robot to move forward according to the next phase
				data.write("Phase 2: Move Forward\n")
				data.write('step,x_center,width, groundSenor1, groundSenor2, groundSenor3\n')
	
	elif phase == 3: #Phase 3 (2) Move towards the Green Block, using proximity senors to stop and record ground senor Values 
			#get calibrated senors values
			ps_values = robot.get_calibrate_prox()
			gs_values = robot.get_ground()
			
			
			
			if stepcounter % n == 0: #helps avoid crashing
				robot.init_camera()
			if stepcounter % n == 1:
				img = np.array(robot.get_camera())
				detections = robot.get_detection(img,0.9)  #use implemented object recognition API and change LED Color depending which object is detected
				for item in detections:
					if item.label == "Green Block":
						data.write("{},{},{},{},{},{}\n".format(step, item.x_center, item.width, gs_values[0], gs_values[1], gs_values[2]))
			stepcounter += 1
			step += 1
			#Once we detect an Object at corresponding Proximity, we change phase
			if(ps_values[0] > MAX_PROX and ps_values[7] > MAX_PROX):
				phase = 4
				step = 0;
				robot.disable_all_led()
				data.write("Phase 3: Turning\n")
				data.write('step,ps0,ps1, ps2, ps3, ps4, ps5, ps6, ps7\n')
	elif phase == 4: #Phase 4 (3) Turn 2 and ¼ Turn
		#enable correct Phase LED
		robot.enable_led(2)
		robot.enable_led(6)
		
		ps_values = robot.get_calibrate_prox()  
		   
		if step < 1600: #1600 Steps are basically exactly 2 and ¼ turn
			data.write("{},{},{},{},{},{},{},{}\n".format(step, ps_values[0], ps_values[1], ps_values[2], ps_values[3], ps_values[4], ps_values[5], ps_values[6], ps_values[7]))
			step += 1
			robot.set_speed(-MAX_SPEED, MAX_SPEED)
		else: #change phase
			phase = 5
			step = 0
			robot.disable_all_led()
			data.write("Phase 4: Wall-following\n")
			data.write('step,ps1, ps2, ps4, ps6, ds_pid\n')
		
	elif phase == 5: #Phase 5 (4) Do Wall-following with PID on the right side
			#enable correct Phase LED	
			robot.enable_led(2)
			# get proximity values
			ps = robot.get_calibrate_prox()
			proxR = (a * ps[0] + b * ps[1] + c * ps[2] + d * ps[3]) / (a+b+c+d);
                      
    		# compute PID response according to IR sensor value
			ds = pid.compute(proxR,PID_WALL_TARGET);      
    		# make the robot turn towards the wall by default    
			ds += .05
			data.write("{},{},{},{},{},{}\n".format(step, ps_values[1], ps_values[2], ps_values[4], ps_values[6], ds))
			speedR = MAX_SPEED + ds
			speedL = MAX_SPEED - ds
			step += 1
			robot.set_speed(speedL,speedR)
			#If an object is detected by the proximity Senors, we change phase
			if (ps[0] > MAX_PROX and ps[7] > MAX_PROX):	
				phase = 6
				stepTEMP = step
				step = 0
				robot.disable_all_led()
	elif phase == 6: #Phase 6 (4) Turn 180°
			
			if step < 300 :
				robot.set_speed(-MAX_SPEED, MAX_SPEED)
				step += 1	
			else:
				phase = 7
				step = stepTEMP
				
	elif phase == 7: #Phase 7 (4) Do Wall-following with PID on the left side
			#enable correct Phase LED
			robot.enable_led(6)
			# get proximity values		
			ps = robot.get_calibrate_prox()
			proxL = (a * ps[7] + b * ps[6] + c * ps[5] + d * ps[4]) / (a+b+c+d);
                      
    		# compute PID response according to IR sensor value
			ds = pid.compute(proxL,PID_WALL_TARGET);      
    		# make the robot turn towards the wall by default    
			ds += .05
			data.write("{},{},{},{},{},{}\n".format(step, ps_values[1], ps_values[2], ps_values[4], ps_values[6], ds))
			step += 1
			speedR = MAX_SPEED - ds
			speedL = MAX_SPEED + ds
			
			robot.set_speed(speedL,speedR)
			#If an object is detected by the proximity Senors, we change phase
			if (ps[0] > MAX_PROX and ps[7] > MAX_PROX):	
				phase = 8
				step = 0
				robot.disable_all_led()
				data.write("Phase 5: Microphone test\n")
				data.write('step, mic1, mic2, mic3\n')

	elif phase == 8: #Phase 8 (5) Record microphones values, while turning 360°
			#enable correct Phase LED	
			robot.enable_led(0)
			robot.enable_led(2)
			robot.enable_led(6)
			
			if step < 250:
				robot.set_speed(-MAX_SPEED, MAX_SPEED)
				microphone_values = robot.get_microphones()
				data.write("{},{},{},{}\n".format(step, microphone_values[0], microphone_values[1], microphone_values[2]))
				step += 1
			else:
				phase = 9
				step = 0
				robot.set_speed(0)
				robot.disable_all_led()
	elif phase == 9: #Phase 9 (5) Task has now been completed!
		#enable correct Phase LED
		robot.enable_led(0)
		robot.enable_led(2)
		robot.enable_led(4)
		robot.enable_led(6)
		robot.enable_body_led()
			
		
		
data.close()
robot.clean_up()
