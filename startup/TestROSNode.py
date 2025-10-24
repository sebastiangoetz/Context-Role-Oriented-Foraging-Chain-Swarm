#! /usr/bin/env python3
from rclpy.node import Node
import rclpy
import threading
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from argos3_ros2_bridge.msg import BlobList, Position, Blob, Load, Grip
import math



class TestROSSupervisor(Node):
    def __init__(self, robotName):
        super().__init__(robotName+"Test")
        
        self.xPos = 0.0
        self.linear = 0.0
        self.angle = 0.0 
        self.grip = False
        self.release = False  
        self.create_subscription(Twist, "/"+robotName+"/cmd_vel", self.cmd_velActuatorCallback, 1)
        #self.create_subscription(Twist, "/"+robotName+"/position", self.odom_sensor_callback, 1)
        self.create_subscription(Grip, "/"+robotName+"/cmd_grip", self.gripActuatorCallback, 1)
        #self.create_subscription(Load, "/"+robotName+"/grip", self.gripActuatorCallback, 1)

        self.publisher_blobs = self.create_publisher(BlobList, "/"+robotName+"/blobList", 1)
        self.publisher_load = self.create_publisher(Load, "/"+robotName+"/load", 1)
        self.publisher_pos = self.create_publisher(Position, "/"+robotName+"/position", 1)
        
        self.message_blobs = BlobList()
        self.message_load = Load()
        self.message_pos = Position()

    def odom_sensor_callback(self, message):
        self.xPos = message.position.x

    def getLinear(self):
        return self.linear
    
    def getAngle(self):
        return self.angle
    
    def getRelease(self):
        return self.release
    
    def odom_sensor_callback(self, message):
        self.xPos = message.pose.pose.position.x
    
    def cmd_velActuatorCallback(self, message):
        self.angle = message.angular.z
        self.linear = message.linear.x

    def gripActuatorCallback(self, message):
        self.grip = message.grip
        self.release = message.release

    def publishLoad(self, load):
        self.message_load.load = load
        self.publisher_load.publish(self.message_load)

    def publishPosition(self, theta):
        self.message_pos.position.x = 0.0
        self.message_pos.position.y = 0.0
        self.message_pos.position.z = 0.0
        self.message_pos.orientation.x = 0.0
        self.message_pos.orientation.y = 0.0
        self.message_pos.orientation.z = math.sin(theta / 2.0)
        self.message_pos.orientation.w = math.cos(theta / 2.0) 

        self.publisher_pos.publish(self.message_pos)

    def publishLight(self, ledColor, angle, distance):

        if ledColor == " ":
            self.message_blobs.n = 0
            self.message_blobs.blobs = []
        else:
            blob = Blob()
                #string color
                #float32 distance
                #float32 angle
            blob.color = ledColor
            blob.distance = angle
            blob.angle = distance
            self.message_blobs.n = 2
            self.message_blobs.blobs = [blob]

        self.publisher_blobs.publish(self.message_blobs)

# if __name__ == '__main__':
#     rclpy.init(args=None)
#     rosNode = TestROSSupervisor("fb_2")
#     threading.Thread(target=lambda: rclpy.spin(rosNode)).start()
#     while True:
#         rosNode.publishLight("red", 0.3, 0.234234)
