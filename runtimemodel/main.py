import json
import threading
import time
from modelUtils.robotSupervisor import *
import rclpy
import socket
from modelImpl.robotModelImpl import *
import re
import sys

# cli arguments for robot name
name = sys.argv[1]
number = re.findall(r'\d+', name)   

global addr, udpServerSocket, bufferSize
addr = None
start = False
bufferSize = 1024
HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 2000+int(number[0])*2 # Port to listen on (non-privileged ports are > 1023)
addrPort = (HOST,PORT)
DIST_TOLERANCE = 0.05
DIST_LOADING = 0.3

# receives Messages from the Webapp, 
# if hello-Message is received the Initialisation Message is created and sent to the Webapp
# otherwise Goal is set
def receiveMessages():
    global start, addr, udpServerSocket    
    while(True): 
        msg = udpServerSocket.recv(bufferSize) # BLOCKS
        if(msg.decode() == "start"):
            start = True
        else:
            msg = json.loads(msg.decode())
            #print(msg)
            if(len(msg)>=2):
                model.implementation(msg["xTarget"],msg["yTarget"], msg["state"], msg["led"])

# creates a Statusmessage in JSON from the Runtimemodel and sends it via Socket to the Webapp
# Message contains Robot with its attributes
def publishMessages():
    global start, udpServerSocket, addr
    while True:
        if(udpServerSocket and start):       
            # robot --> JSON  TODO: make this easier --> Outsource
            d = {}
            robot = model.robots
            # adds only attributes of robot to dict
            d[model.robots.getname()] = {}
            for a in [a for a in dir(robot) if not a.startswith('__') and callable(getattr(robot, a)) and "get" in a] :
                # must call getters, bacause otherwise Area would not return name but only reference               
                # appends attribute name from getter function name without get and attribute Value
                d[robot.getname()][(a[3:])] = getattr(robot, a)()
            #print(robot.getload())
            udpServerSocket.send(str.encode(json.dumps(d)+ "\n"))
            
        time.sleep(0.1) # depends on the performance of your PC

print("Staaaart")     


waiting = StateImpl(1, "waiting", 0.0)
driving = StateImpl(2, "driving", 1.0)
load = StateImpl(5, "load", 0.0, True, False)
unload = StateImpl(6, "unload", 0.0, False, True)

model = ModelImpl(None, [waiting, driving, load, unload])
robot1=RobotImpl(0.0, 0.0, 0.0,0.0, 0.0, name, 1)
robot1.setstate(waiting)
model.addRobot(robot1)



# run robotSupervisor-Node
rclpy.init(args=None)

robotSupervisor = RobotSupervisor(robot1.getname())
threading.Thread(target=lambda: rclpy.spin(robotSupervisor)).start()

# Socket for Connection to Webapp
udpServerSocket= socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
udpServerSocket.connect(addrPort)

# Reveice from Webapp
threading.Thread(target=lambda: receiveMessages()).start()

#Publish to Webapp
threading.Thread(target=lambda: publishMessages()).start()

##### MAPE-Loop
while(True): 
    #Monitor
    robot1.setPos(robotSupervisor.getxPos(), robotSupervisor.getyPos(),robotSupervisor.getzPos(), robotSupervisor.getTheta())
    robot1.setLoad(robotSupervisor.getLoad())
    repulsion = robotSupervisor.getv_repulsion()

    #Analyse - makes the abstraction and checks if goal was Reached
    #model.abstraction(robot1.getid()) 
    robot1.setProximity(robotSupervisor.getProximity()) # Abstraction already made in robotSupervisor
    if(robot1.geDistanceToTarge()>DIST_TOLERANCE):
        robot1.goalReached = False
    else:
        robot1.goalReached = True
    
    # disable collission avoidance when goal is near
    if(robot1.geDistanceToTarge() <= DIST_LOADING or robot1.state != driving):
        repulsion = np.array([0,0])
    
    # Plan - calculates and sets speeds for the robot
    if(not robot1.getgoalReached() and robot1.state == driving):
        robot1.calculateSpeeds(repulsion)

    # if goal reached or state != driving
    elif(robot1.speed != 0.0 or robot1.rotationSpeed != 0.0):
        robot1.speed = robot1.rotationSpeed = 0.0
        robot1.goalReached = False #required to send last velocity command with 0 and 0 to stop the robot    

    #Execute - send speeds to robotSupervisor to publish them to ROS
    if(not robot1.getgoalReached()):
        #print(f'goal: {robot1.getgoalReached()} publish velocity({robot1.speed},{robot1.rotationSpeed})')
        robotSupervisor.publishVelocity(robot1.speed,robot1.rotationSpeed)
    
    if (robot1.speed == 0.0):
        robotSupervisor.publishGripper(robot1.state.getgrip(), robot1.state.getrelease())
    
    robotSupervisor.publishLight(robot1.ledColor) # TODO: with condition, publish only when changed

robotSupervisor.destroy_node()
rclpy.shutdown()

