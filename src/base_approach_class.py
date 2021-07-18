#!/usr/bin/env python3
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
from numpy.core.numeric import Inf
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import ObjectEstimation, ObjectEstimationResponse
from src2_object_detection.srv import FindObject, FindObjectResponse
from src2_approach_services.srv import ApproachChargingStation, ApproachChargingStationResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from srcp2_msgs.srv import LocalizationSrv, SpotLightSrv #AprioriLocationSrv,
import message_filters #for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf import TransformListener, TransformBroadcaster
import tf.transformations as t_
import numpy as np


print_to_terminal = rospy.get_param('approach_base_station_service/print_to_terminal', False)
ROVER_MIN_VEL = rospy.get_param('approach_base_station_service/rover_min_vel', 0.8)
APPROACH_TIMEOUT = rospy.get_param('approach_base_station_service/approach_timeout', 50)
LASER_RANGE = 9
LASER_THRESHOLD = 8.0


class Obstacle:
    """
    Obstacle Definition
    """
    def __init__(self,obstacle, distance):
        self.obstacle = obstacle
        self.distance = distance


class BaseApproachClass:
    """
    Base class for robot approach services with standard driving and sensor
    data processing for performing approach maneuvers
    """
    def __init__(self):
        """
        Written base classes
        """
        pass

    def turn_in_place(self, direction):
        """
        Turn in place clockwise or counter clockwise
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.angular.z = 0.25*direction
        _cmd_publisher.publish(_cmd_message)
        rospy.sleep(0.10)

    def object_distance_estimation(self, object):
        """
        Estimate distance from arg object to the camera of the robot.
        Requires disparity image
        """
        rospy.wait_for_service('object_estimation') #Change the name of the service
        object_estimation_call = rospy.ServiceProxy('object_estimation', ObjectEstimation)
        try:
            object_distance = object_estimation_call(object, self.disparity)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return(object_distance)

    def drive(self,speed, heading):
        """
        Drive function, send args to the cmd velocity topic
        Args:
        speed: linear x velocity of the robot
        heading: angular z velocity of the robot
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = speed
        _cmd_message.angular.z = heading
        _cmd_publisher.publish(_cmd_message)
        rospy.sleep(0.05)

    def command_laser(self, pitch):
        """
        Drive function, send args to the cmd velocity topic
        Args:
        speed: linear x velocity of the robot
        heading: angular z velocity of the robot
        """
        _cmd_publisher = rospy.Publisher("sensor/pitch/command/position", Float64, queue_size = 10 )
        _cmd_message = Float64()
        _cmd_message.data = pitch
        _cmd_publisher.publish(_cmd_message)
        rospy.sleep(0.05)

    def drive_full(self, vx, vy, wz):
        """
        Drive function, send args to the cmd velocity topic
        Args:
        speed: linear x velocity of the robot
        heading: angular z velocity of the robot
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_message.linear.x = vx
        _cmd_message.linear.x = vy
        _cmd_message.angular.z = wz
        _cmd_publisher.publish(_cmd_message)
        rospy.sleep(0.05)

    def stop(self):
        """
        Stop the rover sending zeros cmd velocity
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        _cmd_publisher.publish(_cmd_message)
        rospy.sleep(0.05)


    def check_for_obstacles(self,boxes):
        """
        Check if obstacles exist in the bounding boxes
        """
        self.obstacle_boxes = []
        for box in boxes:
            if box.id == 5:
                self.obstacle_boxes.append(box)

    def check_for_rover(self, boxes):
        """
        Check if obstacles exist in the bounding boxes
        """
        self.rover_boxes = []
        for box in boxes:
            if box.id == 2 or box.id == 3 or box.id == 4:
                self.rover_boxes.append(box)

    def check_for_bin(self, boxes):
        """
        Check if bin exist in the bounding boxes
        """
        for box in boxes:
            if box.id == 6: # bin id == 6 -- regolith id == 12
                self.bin = box


    def check_for_regolith(self, boxes):
        """
        check for grey regolith above bin to perform alignment (face_regolith)
        """
        for box in boxes:
            if box.id == 12: # bin id == 6 -- regolith id == 12
                self.rover = box

    def check_for_base_station(self,boxes):
        """
        Check if base station exist in the bounding boxes
        """
        for box in boxes:
            if box.id == 1:
                self.base = box

    def laser_mean(self):
        """
        Return the average distance from +- 30 deg from the center of the laser
        """
        laser = rospy.wait_for_message("laser/scan", LaserScan)
        _val = 0
        _ind = 0
        for i in laser.ranges[0:99]:
            if not np.isinf(i) and i < LASER_THRESHOLD:
                _val+=i
                _ind+=1
        if _ind != 0:
            range = _val/_ind
            if print_to_terminal:
                print("Laser Range: {}".format(range))
            return range
        else:
            return Inf

    def toggle_light(self, value):
        """
        Service to toggle the lights with float value from zero to one
        as the light internsity (0 being off and 1 high beam)
        """
        rospy.wait_for_service('spot_light')
        toggle_light_call = rospy.ServiceProxy('spot_light', SpotLightSrv)
        try:
            toggle_light_call = toggle_light_call(float(value))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def shutdown(self):
        """
        Shutdown Node
        """
        self.stop()
        rospy.loginfo("Approach Base Station service node is shutdown")
        rospy.sleep(1)

def main():
    try:
        pass
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
