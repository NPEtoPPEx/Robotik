# run the code to generate IR sensor data 
from shutil import move
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
    step = 0
    robot.sleep(3)

    while robot.go_on():

        print("Current Phase : " + str(movePhase) + " Step: " + str(step))
        if movePhase == 1:
            if stepcounter % n == 0 :
                robot.init_camera()
            if stepcounter % n == 1 :
                img = np.array(robot.get_camera())
                detections = robot.get_detection(img, 0.87)
                for object in detections:
                    if object.label == "Epuck":
                        MAX_SPEED = 0.5
                        if object.width >= 55:
                            sendMSG = 1
                            robot.send_msg(sendMSG)
                    else:
                        MAX_SPEED = 1.5
                        sendMSG = 0
                receivedMSG = ""
                while robot.has_receive_msg():
                    receivedMSG = robot.receive_msg()
                if str(receivedMSG) == str(sendMSG) and receivedMSG != 0:
                    movePhase = 2
                    MAX_SPEED = 1.5
        
            stepcounter += 1
        
        
        #drive on linefollowing
            left = robot.get_ground()[0]
            middle = robot.get_ground()[1]
            right = robot.get_ground()[2]
            #no detection -> turn left
            if left > 550 and middle > 550 and right > 550:
                robot.set_speed(MAX_SPEED * (-0.3), MAX_SPEED * (0.3))
            #only right --> turn right
            if left > 550 and middle > 550 and right < 550:
                robot.set_speed(MAX_SPEED * (0.3), MAX_SPEED * (-0.3))
            #only left --> turn leftstraight
            if left < 550 and middle > 550 and right > 550:
                robot.set_speed(MAX_SPEED * (-0.4), MAX_SPEED * (0.4))
            #only left and middle --> straight
            if left < 550 and middle < 550 and right > 550:
                robot.set_speed(MAX_SPEED, MAX_SPEED)
            #all 3--> turn right
            if left < 550 and middle < 550 and right < 550:
                robot.set_speed(MAX_SPEED * (0.3), MAX_SPEED * (-0.3))
            # only midddle --> right-straight
            if left > 550 and middle < 550 and right > 550:
                robot.set_speed(MAX_SPEED * (0.4), MAX_SPEED * (-0.4))
            #middle and right --> straight-right
            if left > 550 and middle < 550 and right < 550:
                robot.set_speed(MAX_SPEED * (0.4), MAX_SPEED*(-0.4))
            #left and right --> straight-right
            if left < 550 and middle > 550 and right < 550:
                robot.set_speed(MAX_SPEED * (0.3), MAX_SPEED*(-0.3))
        elif movePhase == 2:
            
            if step < 30:
                robot.set_speed(0.8, -0.8)
            else:
                movePhase = 3
                step = 0
            step += 1

        elif movePhase == 3:
            if step < 100:
                robot.set_speed(MAX_SPEED, MAX_SPEED)
                step += 1
            else:
                step = 0
                movePhase = 4
        elif movePhase == 4:
            if step < 60:
                robot.set_speed(-0.8, 0.8)
                step += 1
            else:
                movePhase = 5
                step = 0
        elif movePhase == 5:
                left = robot.get_ground()[0]
                middle = robot.get_ground()[1]
                right = robot.get_ground()[2]
                robot.set_speed(MAX_SPEED, MAX_SPEED)
                if left < 550 or middle < 550 or right < 550:
                    robot.set_speed(0,0)
                    movePhase = 1
                    sendMSG = 0
                    receivedMSG = 0

robot.clean_up()