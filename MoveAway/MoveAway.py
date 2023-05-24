# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper
import sys, random, signal
import numpy as np

if __name__ == "__main__":
    """
    if arguments in the command line --> IRL
    no arguemnts --> use Webots
    """
    ip_addr = None
    if len(sys.argv) == 2:
        ip_addr = sys.argv[1]
    
    robot = wrapper.get_robot(ip_addr)
    
    def handler(signum, frame):
        robot.clean_up()
    signal.signal(signal.SIGINT, handler)   

    robot.init_ground()
    robot.init_client_communication()
    robot.init_camera("img")
    robot.initiate_model()
    sendMSG = 0
    stepcounter = 0
    n = 10
    MAX_SPEED = 1.5
    movePhase = 1 # 1 stay on line, 2 move to the right, 3 move back to line
    # wait 3 seconds before startin
    robot.sleep(3)

    while robot.go_on():
        if movePhase == 1:
            if stepcounter % n == 0 :
                robot.init_camera()
            if stepcounter % n == 1 :
                img = np.array(robot.get_camera())
                detections = robot.get_detection(img)
                for object in detections:
                    if object.label == "Epuck":
                        print("detected")
                        robot.set_speed(0, 0)
                        sendMSG = 1
                    else:
                        sendMSG = 0
                    
                
                receivedMSG = ""
                while robot.has_receive_msg():
                    receivedMSG = robot.receive_msg()
                if str(receivedMSG) == str(sendMSG) and receivedMSG != 0:
                    print("Change")
                    movePhase = 2
        
            stepcounter += 1
        
        
        #drive on linefollowing
            left = robot.get_ground()[0]
            middle = robot.get_ground()[1]
            right = robot.get_ground()[2]
    	
            if left <= 500 and middle <= 500 and right >500:
                robot.set_speed(MAX_SPEED)
            elif left <= 500 and middle <= 500 and right <= 500:
                robot.set_speed(MAX_SPEED * 0.15, MAX_SPEED * (-0.15))
            elif left >= 500 and middle >= 500 and right <= 500:
                robot.set_speed(MAX_SPEED * 0.15, MAX_SPEED * (-0.15))
            elif left >= 500 and middle >= 500 and right >= 500:
                robot.set_speed(MAX_SPEED * (-0.15), MAX_SPEED * (0.15))
        elif movePhase == 2:
        #make it turn around 40° then drive 300 step
        #afterwards go movePhase 3
            s = 0
        elif movePhase == 3:
            #turn 40* to the left then drive until you find the line, once you do 
            #switch back to movePhase 1
            s = 0
robot.clean_up()