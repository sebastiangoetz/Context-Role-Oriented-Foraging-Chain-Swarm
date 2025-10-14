import json
import threading
import time
import re
from modelUtils.robotSupervisor import *
import rclpy
import socket
import sys


name = sys.argv[1]
number = re.findall(r'\d+', name)

global addr, udpServerSocket, bufferSize
start = False
addr = None
bufferSize = 1024
HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 2000+int(number[0])*2+1  # Port to listen on (non-privileged ports are > 1023)
addrPort = (HOST,PORT)

# run robotSupervisor-Node
rclpy.init(args=None)
robotSupervisor = RobotMessage(name)
threading.Thread(target=lambda: rclpy.spin(robotSupervisor)).start()

# Socket for Connection to Webapp
udpServerSocket= socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
udpServerSocket.connect(addrPort)

def getAngleDif(a,b):
    return math.pi - abs(abs(a - b) - math.pi); 

blobs_old = []
# ##### MAPE-Loop
while(True):
    blobs = robotSupervisor.getBlobs()
    if blobs_old != [] and blobs == []: #send a last nothing message, before stop sending
        print("last message")
        udpServerSocket.send(str.encode(json.dumps([["nothing", 0, 0]])+ "\n"))
    if blobs != []:
        messageList = []
        filteredBlobs= {}  # dict: color -> blob mit kleinster Distanz

        for blob in blobs:
            # Filter out blobs behind the robot
            if -1.3 < blob.angle < 1.3:
                # take the blob, if the color is not in the dict or if it is closer than the former blob with this color
                if blob.color not in filteredBlobs or blob.distance < filteredBlobs[blob.color].distance:
                    filteredBlobs[blob.color] = blob

        # interpret the sorted blobs to messages
        for color, blob in filteredBlobs.items():
            angle = blob.angle + robotSupervisor.getTheta()
            yPos = math.sin(angle) * (blob.distance / 100)
            xPos = math.cos(angle) * (blob.distance / 100)
            xAbs = robotSupervisor.getxPos() + xPos
            yAbs = robotSupervisor.getyPos() + yPos

            if color == "red":
                messageList.append(["Prey detected", xAbs, yAbs])
            elif color == "yellow" and not any(msg[0] == "Robot with Load detected" for msg in messageList):
                messageList.append(["Robot with Load detected", xAbs, yAbs])
            elif color == "magenta" and not any(msg[0] == "Chainmember detected" for msg in messageList):
                messageList.append(["Chainmember detected", xAbs, yAbs])
            elif color == "white":
                messageList.append(["Joiner with Loading State detected", xAbs, yAbs])
            elif color == "green":
                messageList.append(["Joiner detected", xAbs, yAbs])
        if messageList == []: 
            messageList.append(["nothing", 0, 0])
        print(json.dumps(messageList))
        udpServerSocket.send(str.encode(json.dumps(messageList)+ "\n"))
    blobs_old = blobs
    time.sleep(0.1)

robotSupervisor.destroy_node()
rclpy.shutdown()

