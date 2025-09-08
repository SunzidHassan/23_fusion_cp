#!/usr/bin/env python3

"""
Import necessary packages
"""
# ROSPY
import rospy
import sys

# Turtlebot messages
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
# from nav_msgs.msg import Odometry
import tf

# Calculations
import math
import os
import pandas as pd
import numpy as np
# quaternion transformation
from scipy.spatial.transform import Rotation
# plotting
import matplotlib.pyplot as plt
# image data processing
import cv2 as cv

#Yolo vision model
from ultralytics import YOLO

#Date time
import datetime

"""
Parameter Initializations
"""
# olfaction_prompt = input('Deactivate olfaction sensing? [Y/n]: ')
# if olfaction_prompt == 'Y' or olfaction_prompt == 'y':
#     olfaction_model = False

# vision_prompt = input('Deactivate vision sensing? [Y/n]: ')
# if vision_prompt == 'Y' or vision_prompt == 'y':
#     vision_model = False
            
# chemical sensor threshold
chmThr = 500

# chmThr = 100

# trajectory
path_x = []
path_y = []

# search area range
x_min = None
y_min = None
x_max = None
y_max = None

classes_to_filter = None

"""
Server is the main class that read sensor data, process the data with DNN,
and command the robot.
"""
class Server:
    def __init__(self) -> None:
        # Source position 1
        # self.source_position_x = float(input('Enter Goal x-Position: '))
        # self.source_position_y = float(input('Enter Goal y-Position: '))
        self.source_position_x = 3.0
        self.source_position_y = 1.5
        self.dist_threshold = 0.7 # Todo: reduce the distance threshold after vision-laser fusion.
        self.dist = 0.0

        # Sensor variables
        self.chemical = 0.0
        self.airFlow = 0.0
        self.localWindDir = 0.0
        self.laser = None

        self.rotDegree = 0.0
        self.rotEulerDegree = 0.0

        # laser variables
        self.laserAngleFront_0, self.laserAngleSLeft_15, self.laserAngleLeft_60, self.laserAngleBack_120, self.laserAngleRight_180, self.laserAngleSRight_225 = 0, 15, 60, 120, 180, 225

        # Laser scan range threshold
        self.laserAngleThr0 = 1.0
        self.laserAngleThr2 = 0.75
        self.laserAngleThr3 = 0.5

        self.obstacle_boolFront = True
        self.obstacle_boolSlightRgtLft = True
        self.obstacle_boolRgtLft = True

        self.targetHeading = True
        self.loop = 0

        # Vision variables
        self.imageWidth = 640
        self.sequence = None
        self.imgDecode = None

        self.vizObjx = None
        # self.vizObjy = None
        # self.vizObjw = None
        # self.vizObjh = None

        self.preVizObjx = None
        # self.preVizObjy = None
        # self.preVizObjw = None
        # self.preVizObjh = None

        # Behavior variables
        self.behavior_switch = 'Out'
        self.Sign = 1
        self.findDir = 0

        # target heading in degrees and global frame
        self.target_heading = 0.0

        # robot operating time
        self.timeRate = 30
        self.time = 0.0

        # chemical plume detection
        self.detection_flag = 0             # 0 for non-detection, 1 for detection
        self.pre_chem = 0.0

        # dnn parameters
        self.ux = 0.0
        self.uy = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.plume_non_detection = 0.0

        # save data into lists
        self.path_x = []
        self.path_y = []
        self.ux_list = []
        self.uy_list = []
        self.vx_list = []
        self.vy_list = []
        self.time_list = []
        self.detection_flag_list = []
        self.behavior_switch_list = []
        self.behavior_list = []
        self.plume_non_detection_list = []

        # Move parameter
        self.move = 0.0

    def imageCapture(self, imageInput):
        image = np.asarray(bytearray(imageInput.data), dtype="uint8")
        self.imgDecode = cv.imdecode(image, cv.IMREAD_COLOR)
        self.sequence = imageInput.header.seq
        self.visualObjectDetection()

    
    def visualObjectDetection(self):
        if self.sequence % 180 == 0:
            trainedModel = YOLO('/home/lw-lab/catkin_ws/src/my_robot_controller/scripts/YOLOV8_HumidifierSmokeDetectionWeight/best.pt')
            result = trainedModel(self.imgDecode)
            for box in result[0].boxes.xyxy:
                x1, y1, x2, y2 = box
                self.vizObjx = int((x1+x2)/2)
            self.preVizObjx = self.vizObjx
        
        

    def sensorCallback(self, msg):
        # self.chemical = 0
        self.chemical = msg.illumination
        self.airFlow = msg.cliff
        self.localWindDir = msg.sonar

       
    def laserCallback(self, msg):
        self.laser = msg

    def tflistener(self, listener):
        while not rospy.is_shutdown():
            try:
                (self.trans,self.rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.xPosition = self.trans[0]
            self.yPosition = self.trans[1]
            self.zOrientation = self.rot[2]
            self.wOrientation = self.rot[3]
            self.sensorConversion()

    
    def sensorConversion(self):
        # convert quarternion to Euler degree
        Rot = Rotation.from_quat([0, 0, self.zOrientation, self.wOrientation])
        self.rotEulerDegree = Rot.as_euler('xyz', degrees=True)[2]

        # convert euler degree to rotation degree
        if self.rotEulerDegree < 0:
            self.rotDegree = -self.rotEulerDegree
        else:
            self.rotDegree = 360 - self.rotEulerDegree

        if self.localWindDir > 180:
            self.localWindDir -= 360

        self.prelocalWindDir = self.localWindDir
        self.globalWindDir = self.localWindDir - self.rotEulerDegree
        if self.globalWindDir > 180:
            self.globalWindDir -= 360

        # # temp sensor fixing
        # self.globalWindDir = 0

        self.globalWindBlowDir = self.globalWindDir + 180

        # # convert local wind direction to global wind direction
        # globalWindDir = self.localWindDir + rotDegree
        # if globalWindDir > 360:
        #     globalWindDir -= 360

        # # convert global wind direction to wind blow direction
        # globalWindBlowDir = globalWindDir + 180
        # if globalWindBlowDir > 360:
        #     globalWindBlowDir -= 360
        
        # # convert the global wind blow diretcion to align with the global coordination
        # if globalWindBlowDir <= 180:
        #     globalWindBlowDir = - globalWindBlowDir
        
        # if globalWindBlowDir > 180:
        #     globalWindBlowDir = 360 - globalWindBlowDir

        self.setSensorFlag()




    # check the current robot behaviors
    # 0: find plumes, i.e., zigzag
    # 1: maintain in, i.e., DNN or moth inspired
    # 2: maintain out, i.e., cross wind movement
    # 3: vision towards target object movement
    # To-do: obstacle avoidance behavior
    def check_behavior(self, behavior_switch):
        # default behavior is zigzag
        behavior = 'Out'
        if behavior_switch == 'switch_to_obstacleAvoid':
            behavior = 'ObstacleAvoidance'
        elif behavior_switch == 'switch_to_visionNav':
            behavior = 'Vision'
        # if behavior_switch == 'switch_to_zigzag':
        #     behavior = 'Find'
        elif behavior_switch == 'switch_to_in':
            behavior = 'In'
        elif behavior_switch == 'switch_to_out':
            behavior = 'Out'

        return behavior


    def pid_heading(self, target_heading, current_heading):
        kp = 0.03

        error = current_heading - target_heading

        if error < -180:
            error += 360
        if error > 180:
            error -= 360

        pid_output = kp * error

        if pid_output > 0.3:
            pid_output = 0.3

        if pid_output < -0.3:
            pid_output = -0.3

        return -pid_output
    

    def setSensorFlag(self):
        self.laserAngleFront_0, self.laserAngleSLeft_15, self.laserAngleSLeft_30, self.laserAngleLeft_60, self.laserAngleBack_120, self.laserAngleRight_180, self.laserAngleSRight_210, self.laserAngleSRight_225 = 0, 15, 30, 60, 120, 180, 210, 225

        self.obstacle_boolFront = self.laser.ranges[self.laserAngleFront_0] < self.laserAngleThr0
        self.obstacle_boolSlightRgtLft = (self.laser.ranges[self.laserAngleSLeft_15] < self.laserAngleThr0) or (self.laser.ranges[self.laserAngleSRight_225] < self.laserAngleThr0)
        self.obstacle_boolHalfRgtLft = (self.laser.ranges[self.laserAngleSLeft_30] < self.laserAngleThr2) or (self.laser.ranges[self.laserAngleSRight_210] < self.laserAngleThr2)
        self.obstacle_boolRgtLft = (self.laser.ranges[self.laserAngleLeft_60] < self.laserAngleThr3) or (self.laser.ranges[self.laserAngleRight_180] < self.laserAngleThr3)
        # self.obstacle_boolSlightRgtLft = (0.01 < self.laser.ranges[self.laserAngleSLeft_15] < self.laserAngleThr2) or (0.01 < self.laser.ranges[self.laserAngleSRight_225] < self.laserAngleThr2)
        # self.obstacle_boolRgtLft = (0.01 < self.laser.ranges[self.laserAngleLeft_60] < self.laserAngleThr3) or (0.01 < self.laser.ranges[self.laserAngleRight_180] < self.laserAngleThr3)

        if self.obstacle_boolFront or self.obstacle_boolSlightRgtLft or self.obstacle_boolHalfRgtLft or self.obstacle_boolRgtLft:
            self.detection_flag = 'obstacle_detected'
        # determine whether the robot detects plume source object in vision
        elif self.preVizObjx is not None:
            self.detection_flag = 'vision_detected'
        # if object is not detected in vision, trace plume
        else:
            # determine whether the robot detects chemical plumes
            if self.chemical < chmThr:
                self.detection_flag = 'chemical_not_detected'
            # if current plume concentration is greater than previous reading
            else:
                if self.chemical > self.pre_chem:
                    self.detection_flag = 'chemical_detected'
                else:
                    self.detection_flag = 'chemical_detected'     # Should be 'chemical_not_detected'
                    # self.detection_flag = 'chemical_not_detected'

            self.pre_chem = self.chemical

            # update non-detection time if plume not detected
            if self.detection_flag == 'chemical_not_detected':
                self.plume_non_detection += 0.02
        self.navigate()


    def navigate(self):
        self.move = Twist()  # Creates a Twist message type object
        rate = rospy.Rate(self.timeRate)

        # determine the current behaviors
        behavior = self.check_behavior(behavior_switch=self.behavior_switch)

        # zigzag behavior (behavior_switch = 0)
        if behavior == 'Find':
            if self.detection_flag == 'chemical_not_detected':
                if self.yPosition > y_max or self.yPosition < y_min:
                    if self.yPosition > 0:
                        self.Sign = -1
                    else:
                        self.Sign = 1

                # determine whether the robot moves toward the downwind area
                if self.xPosition < x_min:
                    self.findDir = 0     # downwind
                if self.xPosition > x_max:
                    self.findDir = 1     # upwind

                if self.findDir == 1:
                    self.target_heading = 0 + self.Sign * 110
                if self.findDir == 0:
                    self.target_heading = 0 + self.Sign * 70
            # change to obstacle avoidance, vision or in behavior
            elif self.detection_flag == 'obstacle_detected':
                self.behavior_switch = 'switch_to_obstacleAvoid'
            elif self.detection_flag == 'vision_detected':
                self.behavior_switch = 'switch_to_visionNav'
            elif self.detection_flag == 'chemical_detected':
                self.behavior_switch = 'switch_to_in'


        # track-in behavior (behavior_switch = 1)
        if behavior == 'In':
            self.targetHeading = True
            # move against wind direction
            if self.detection_flag == 'chemical_detected':
                self.target_heading = self.globalWindDir
            elif self.detection_flag == 'obstacle_detected':
                self.behavior_switch = 'switch_to_obstacleAvoid'
            elif self.detection_flag == 'vision_detected':
                self.behavior_switch = 'switch_to_visionNav'
            elif self.detection_flag == 'chemical_not_detected':
                self.behavior_switch = 'switch_to_out'


        # track-out behavior (behavior_switch = 2)
        if behavior == 'Out':
            self.targetHeading = True
            # move across wind direction
            if self.detection_flag == 'chemical_not_detected':
                self.target_heading = self.globalWindDir + 90
            elif self.detection_flag == 'obstacle_detected':
                self.behavior_switch = 'switch_to_obstacleAvoid'
            elif self.detection_flag == 'vision_detected':
                self.behavior_switch = 'switch_to_visionNav'
            elif self.detection_flag == 'chemical_detected':
                self.behavior_switch = 'switch_to_in'

        # vision-led behavior (behavior_switch = 3)
        if behavior == 'Vision':
            self.targetHeading = False
            if self.preVizObjx is not None:
                self.move.linear.x = 0.08
                # if object is in the left of the image, move right, and vice versa
                if float(self.preVizObjx) < self.imageWidth/2:
                    self.move.angular.z = 0.03
                else:
                    self.move.angular.z = -0.03
            if self.detection_flag == 'obstacle_detected':
                self.behavior_switch = 'switch_to_obstacleAvoid'
            elif self.detection_flag == 'chemical_detected':
                self.behavior_switch = 'switch_to_in'
            elif self.detection_flag == 'chemical_not_detected':
                self.behavior_switch = 'switch_to_out'

        # obstacle avoidance behavior

        if behavior == 'ObstacleAvoidance':
            if self.detection_flag == 'obstacle_detected':
                thr1 = 0.8 # Laser scan range threshold
                thr2 = 0.6
                # no obstacles front, slightly left or slightly right
                if self.laser.ranges[0]>thr1 and self.laser.ranges[25]>thr2 and self.laser.ranges[215]>thr2:
                    self.targetHeading = True
                    self.loop = 1
                    if self.laser.ranges[25] > self.laser.ranges[215]:
                        self.target_heading = self.globalWindDir + 90
                    else:
                        self.target_heading = self.globalWindDir - 90
                    # self.move.linear.x = 0.05 # go forward (linear velocity)
                    # self.move.angular.z = 0.0 # do not rotate (angular velocity)
                # no obstacles slightly left
                elif self.laser.ranges[25] > thr1:
                    self.targetHeading = False
                    self.move.linear.x = 0.0
                    self.move.angular.z = 0.5
                    self.loop = 2
                    if self.laser.ranges[0]>thr1 and self.laser.ranges[25]>thr2 and self.laser.ranges[215]>thr2:
                        self.targetHeading = True
                        self.move.linear.x = 0.08
                        self.move.angular.z = 0.0
                        self.loop = 3
                        self.target_heading = self.globalWindDir + 90
                # no obstacles slightly right
                elif self.laser.ranges[215] > thr1:
                    self.targetHeading = False
                    self.move.linear.x = 0.0
                    self.move.angular.z = -0.5
                    self.loop = 4
                    if self.laser.ranges[0]>thr1 and self.laser.ranges[25]>thr2 and self.laser.ranges[215]>thr2:
                        self.targetHeading = True
                        self.move.linear.x = 0.08
                        self.move.angular.z = 0.0
                        self.loop = 5
                        self.target_heading = self.globalWindDir - 90
                # obstacle in both sligtly left and right and front
                else:
                    # # if both slightly light and right is less than thresholds
                    # if self.laser.ranges[25] < thr2 and self.laser.ranges[215] < thr2:
                    #     self.targetHeading = False
                    #     self.loop = 10
                    #     if self.laser.ranges[60] > self.laser.ranges[180]:
                    #         self.move.linear.x = 0.0
                    #         self.move.angular.z = 0.5
                    #     else: 
                    #         self.move.linear.x = 0.0
                    #         self.move.angular.z = -0.5
                    
                    # if space in slight left is greater than slight right
                    if self.laser.ranges[25] > self.laser.ranges[215]:
                        self.targetHeading = False
                        self.move.linear.x = 0.0
                        self.move.angular.z = 0.5
                        self.loop = 6
                        if self.laser.ranges[0]>thr1 and self.laser.ranges[25]>thr2 and self.laser.ranges[215]>thr2:
                            self.targetHeading = True
                            self.move.linear.x = 0.08
                            self.move.angular.z = 0.0
                            self.loop = 7
                            self.target_heading = self.globalWindDir + 90
                    # if space in slight right is greater than slight left
                    elif self.laser.ranges[25] < self.laser.ranges[215]:
                        self.targetHeading = False
                        self.move.linear.x = 0.0
                        self.move.angular.z = -0.5
                        self.loop = 8
                        if self.laser.ranges[0]>thr1 and self.laser.ranges[25]>thr2 and self.laser.ranges[215]>thr2:
                            self.targetHeading = True
                            self.move.linear.x = 0.08
                            self.move.angular.z = 0.0
                            self.loop = 9
                            self.target_heading = self.globalWindDir - 90

            elif self.detection_flag == 'vision_detected':
                self.behavior_switch = 'switch_to_visionNav'
            elif self.detection_flag == 'chemical_detected':
                self.behavior_switch = 'switch_to_in'
            elif self.detection_flag == 'chemical_not_detected':
                self.behavior_switch = 'switch_to_out'

        # PID heading control after the heading is obtained
        if self.targetHeading == True:
            if self.target_heading > 180:
                self.target_heading -= 360
            if self.target_heading < -180:
                self.target_heading += 360
            self.move.angular.z = self.pid_heading(target_heading=self.target_heading, current_heading=self.rotEulerDegree)
            self.move.linear.x = 0.08

        pub.publish(self.move) # publish the move object
        self.printInfo()
        rate.sleep()
    

    def printInfo(self):
        behavior = self.check_behavior(behavior_switch=self.behavior_switch)

        print('Laser reading in front: {}, front threshold{}'.format(self.laser.ranges[self.laserAngleFront_0], self.laserAngleThr0))
        print('Laser reading slight Left: {}, slight Right: {}, threshold: {}'.format(round(self.laser.ranges[self.laserAngleSLeft_15],1), round(self.laser.ranges[self.laserAngleSRight_225],1), self.laserAngleThr2))
        print('Laser reading Left: {}, Right: {}, threshold: {}'.format(round(self.laser.ranges[self.laserAngleLeft_60],1), round(self.laser.ranges[self.laserAngleRight_180],1), self.laserAngleThr3))
        print('Obstacle bool front: {}, slight right-left: {}, right-left: {}, obstacle condition: {}'.format(self.obstacle_boolFront, self.obstacle_boolSlightRgtLft, self.obstacle_boolRgtLft, self.obstacle_boolFront or self.obstacle_boolSlightRgtLft or self.obstacle_boolRgtLft))
        print('Obstacle loop: {}'.format(self.loop))
        print('**')
        print('Time: {}'.format(self.time))
        print('x-Position: {}'.format(round(self.xPosition,1)))
        print('y-Position: {}'.format(round(self.yPosition, 1)))
        # print('rotEulerDegree: {}'.format(self.rotEulerDegree))
        print('rotDegree: {}'.format(round(self.rotDegree, 1)))
        print('**')
        print('Chemical Sensor: {} ppm'.format(round(self.chemical, 1)))
        print('Airflow Speed: {} m/s'.format(round(self.airFlow, 1)))
        print('Local Airflow Direction: {} degrees'.format(round(self.localWindDir, 1)))
        # print('Global Airflow Blow Direction: {} degrees'.format(self.globalWindBlowDir))
        print('Global Airflow Direction: {} degrees'.format(round(self.globalWindDir, 1)))
        #print('Avg Global Airflow Direction: {} degrees'.format(self.AvgGlobalWindDir))
        print('**')
        print('PID Output: {}'.format(round(self.move.angular.z, 1)))
        print('Target Heading: {}'.format(round(self.target_heading, 1)))
        print('Current Heading: {}'.format(round(self.rotEulerDegree, 1)))
        print('Current linear speed: {}'.format(self.move.linear.x))
        print('Current angualr speed: {}'.format(self.move.angular.z))
        print('**')
        print('Current Behavior: {}'.format(behavior))
        print('Detection flag: {}'.format(self.detection_flag))
        print('Distance to the Source: {}'.format(round(self.dist, 1)))
        print('**')
        print('X value of detected object: {}'.format(self.preVizObjx))
        print(float(self.preVizObjx) < self.imageWidth/2 if self.preVizObjx is not None else 'False')

        print('-------------------------------------------')
        self.tracePath()


    def tracePath(self):
        if self.time >= (1/self.timeRate):   # time interval depends on rospy.rate
            
            # record parameters
            # self.ux = self.airFlow * math.cos(self.globalWindBlowDir * math.pi / 180)
            # self.uy = self.airFlow * math.sin(self.globalWindBlowDir * math.pi / 180)

            # self.vx = math.cos(self.rotEulerDegree * math.pi / 180)
            # self.vy = math.sin(self.rotEulerDegree * math.pi / 180)
            
            self.time_list.append(self.time)
            self.path_x.append(self.xPosition)
            self.path_y.append(self.yPosition)
            # self.ux_list.append(self.ux)
            # self.uy_list.append(self.uy)
            # self.vx_list.append(self.vx)
            # self.vy_list.append(self.vy)
            # self.plume_non_detection_list.append(self.plume_non_detection)
            # self.detection_flag_list.append(self.detection_flag)
            # self.behavior_switch_list.append(self.behavior_switch)
            self.behavior_list.append(self.check_behavior(behavior_switch=self.behavior_switch))

            self.save_data()

            # TO-DO: add plot


        # update the running time
        self.time += (1/30)       # time interval depends on rospy.rate

    def save_data(self):
        month = datetime.datetime.now().month
        day = datetime.datetime.now().day
        hour = datetime.datetime.now().hour
        minute = datetime.datetime.now().minute

        self.dist = math.sqrt((self.xPosition - self.source_position_x)**2 + (self.yPosition - self.source_position_y)**2)
        if self.dist < self.dist_threshold or self.time > 200:
            savePrompt = input('Save the path? [Y/n]: ')
            if savePrompt == 'Y' or savePrompt == 'y':
                data = {'time': self.time_list, 'x': self.path_x, 'y': self.path_y, 'behavior': self.behavior_list}
                # data = {'time': self.time_list, 'x': self.path_x, 'y': self.path_y, 'ux': self.ux_list, 'uy': self.uy_list, 'vx': self.vx_list, 
                #         'vy': self.vy_list, 'pnd': self.plume_non_detection_list}       
                df = pd.DataFrame(data)
                print('Save csv File...')
                df.to_csv(f'/home/lw-lab/LLMSim/24-{month}-{day}-{hour}:{minute}.csv', index=False)
                sys.exit(0)
            else:
                sys.exit(0)



def main():
    rospy.init_node('moth_inspired_with_vision_navigation_node')  # Initializes a node

    server = Server()

    # rospy.Subscriber("/odom", Odometry, server.positionCallback)
    rospy.Subscriber("/sensor_state", SensorState, callback=server.sensorCallback)
    rospy.Subscriber("/scan", LaserScan, callback=server.laserCallback)
    rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=server.imageCapture)
    listener = tf.TransformListener()
    server.tflistener(listener)

    rospy.spin()  # Loops infinitely until someone stops the program execution


if __name__ == '__main__':
    try:
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        main()

    except rospy.ROSInterruptException:
        pass
