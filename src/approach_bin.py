#!/usr/bin/env python3
"""
Created on Sun May 17 22:58:24 2020

@author: Chris Tatsch
"""
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
from src2_approach_services.srv import ApproachBin, ApproachBinResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from srcp2_msgs.srv import LocalizationSrv, SpotLightSrv  # AprioriLocationSrv,

from base_approach_class import BaseApproachClass


import message_filters  # for sincronizing time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf import TransformListener, TransformBroadcaster
import tf.transformations as t_
import numpy as np


print_to_terminal = rospy.get_param('approach_bin/print_to_terminal', True)
ROVER_MIN_VEL = rospy.get_param('approach_bin/rover_min_vel', 0.8)
APPROACH_TIMEOUT = rospy.get_param('approach_bin/approach_timeout', 50)
# LASER_RANGE = rospy.get_param('approach_bin/laser_range',  2.0)
LASER_RANGE = 5.50
LASER_CLOSE_RANGE = 2.75
ROTATIONAL_SPEED = rospy.get_param('approach_bin/rotational_speed',  0.25)


class Obstacle:
    """
    Obstacle Definition
    """

    def __init__(self, obstacle, distance):
        self.obstacle = obstacle
        self.distance = distance


class ApproachbinService(BaseApproachClass):
    """
    Service to find the bin and approach it using visual servoing
    """

    def __init__(self):
        """
        """
        self.robot_name = rospy.get_param("robot_name")
        rospy.on_shutdown(self.shutdown)
        self.rover = False
        self.obstacles = []
        self.timeout = 60
        self.boxes = DetectedBoxes()
        self.distance_threshold = LASER_CLOSE_RANGE
        rospy.sleep(2)
        rospy.logerr("APPROACH BIN. Approach bin service node is running")
        s = rospy.Service('approach_bin_service', ApproachBin,
                          self.approach_bin_handle)
        rospy.spin()

    def image_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        self.disparity_sub = rospy.Subscriber("disparity", DisparityImage, self.disparity_callback)

    def image_unregister(self):
        rospy.logerr("APPROACH BIN. Approach Bin Unregister")
        self.disparity_sub.unregister()


    def image_callback(self, img):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.left_image = img


    def disparity_callback(self, disparity):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.obstacles = []
        self.disparity = disparity
        ##Calling the service
        rospy.wait_for_service('/find_object')
        _find_object =rospy.ServiceProxy('/find_object', FindObject)
        try:
            _find_object = _find_object(robot_name = self.robot_name)
        except rospy.ServiceException as exc:
            rospy.logerr("APPROACH BIN.Service did not process request: " + str(exc))
        print(_find_object)
        self.boxes = _find_object.boxes
        self.check_for_obstacles(self.boxes.boxes)
        for obstacle in self.obstacle_boxes:
            dist = self.object_distance_estimation(obstacle)
            self.obstacles.append(Obstacle(obstacle, dist.object_position.point.z))

    def approach_bin_handle(self, req):
        """
        Service for approaching the bin in the SRC qualification
        """

        self.image_subscriber()
        response = self.search_for_bin()
        rospy.logerr("APPROACH BIN. Service Started")
        self.image_unregister()
        return response

    def search_for_bin(self):
        """
        Turn in place to check for bin
        """
        _range = 0.0
        search = False
        for i in range(150):
            self.turn_in_place(-1)
            self.check_for_bin(self.boxes.boxes)
            if self.rover:
                rospy.logerr("APPROACH BIN. Bin found")
                if print_to_terminal:
                    print(self.rover)
                self.stop()
                _range, search = self.approach_bin()
                if print_to_terminal:
                    print("Rover approach to bin was: {}"
                          "and the laser distance is {}".format(search, _range))
                if search == True:
                    self.face_regolith()
                    # self.hauler_dump(2.0)
                self.stop()
                break
        response = ApproachBinResponse()
        resp = Bool()
        resp.data = search
        response.success = resp
        bin_range = Float64()
        bin_range.data = float(_range)
        response.range = bin_range
        if search == True:
            rospy.logerr("APPROACH BIN. Rover approached bin")
        else:
            rospy.logerr("APPROACH BIN. Bin was not found when running turn in place maneuver or timeout")
        self.rover = False  # reset flag variable
        return response

    def approach_bin(self):
        """
        Visual approach the bin,
        !!Need to improve robustness by adding some error check and obstacle avoidance
        """
        rospy.logerr("APPROACH BIN. Starting.")
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()
        toggle_light_ = 1
        while True:
            curr_time = rospy.get_time()
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("APPROACH BIN. Timeout in approach bin service.")
                return 0.0, False

            if toggle_light_ == 1:
                self.toggle_light(10)
                toggle_light_ = 0
            else:
                self.toggle_light(0)
                toggle_light_ = 1

            self.check_for_bin(self.boxes.boxes)

            # Check for Rover
            self.check_for_rover(self.boxes.boxes)
            if self.rover_boxes:
                for object_ in self.rover_boxes:
                    dist = self.object_distance_estimation(object_).object_position.point.z
                    self.check_for_rover(self.boxes.boxes)
                    if dist < 6.0:
                        rospy.sleep(10)

            laser = self.laser_mean()
            rospy.logerr("APPROACH BIN. Distance Laser")
            print(laser)

            print("Base speed : ", speed/2)
            speed = ROVER_MIN_VEL

            x_mean_base = float(self.rover.xmin+self.rover.xmax)/2.0-320
            rotation_speed = -x_mean_base/840

            if laser < self.distance_threshold:
                rospy.logerr("APPROACH BIN. Approached to target (median distance). Stopping.")
                self.stop()
                break
            elif laser < LASER_RANGE:
                rospy.logerr("APPROACH BIN. Closer from target. Drive.")
                self.check_for_excavator(self.boxes.boxes)
                x_mean = float(self.rover.xmin+self.rover.xmax)/2.0-320
                if np.abs(x_mean) >= 100:
                    self.stop()
                    self.face_regolith()
                else:
                    rospy.logerr("APPROACH BIN. Far from target. Drive.")
                    print("Drive speed : ", speed/4)
                    print("Drive rotational speed : ", rotation_speed/8)
                    self.drive(speed/4, rotation_speed/8)
                    # if within LASER_RANGE, approach to half that distance ~2m? very slowly
            else:
                rospy.logerr("APPROACH BIN. Far from target. Drive.")
                print("Drive speed : ", speed/2)
                print("Drive rotational speed : ", rotation_speed/2)
                self.drive(speed/2, rotation_speed/2)

            # if laser < 5:
            #     minimum_dist = 3.0
            # speed = minimum_dist/10.0
            # print("SPEED : ", speed)
            # rotation_speed = -x_mean_base/840+turning_offset+0.5*turning_offset_i
            # # print("ENTERING DRIVE")
            # self.drive(speed, rotation_speed)
            # print("Distance Inference")
            # print(self.object_distance_estimation(self.rover).object_position.point.z)
            # print("Distance Laser")
            # print(laser)
            # # (self.rover.xmax-self.rover.xmin) > 200
            # if laser < LASER_RANGE and laser != 0.0:
            #     self.stop()
            #     self.face_regolith()
            #     # if within LASER_RANGE, approach to half that distance ~2m? very slowly
            #     print("******DOING SLOW/CLOSE BIN APPROACH********")
            #     self.drive(speed/4, 0.0)
            #     if laser < LASER_RANGE/2 and laser !=0.0:
            #         print("+++++++++++++++DID CLOSE APPROACH AND IS WITHIN 2.25m, ready to dump")
            #         self.stop()
            #         break

        rospy.sleep(0.1)
        rospy.logerr("APPROACH BIN. Beginning to DUMP.")
        self.drive(0.1,0.0)
        rospy.sleep(5)
        self.hauler_dump(1.57)
        rospy.sleep(10)
        self.hauler_dump(3.14)
        self.stop()
        self.hauler_bin_reset(0.0)

        self.stop()
        return self.laser_mean(), True

    def face_regolith(self):
        """
        Service to align the rover to the bin using
        bounding boxes from inference node
        """
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()

        while True:
            curr_time = rospy.get_time() # Timeout break:
            rospy.logerr("APPROACH BIN. Trying to face excavator")
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("APPROACH BIN. Timeout in FACE Regolith ")
                break

            self.check_for_bin(self.boxes.boxes)
            x_mean = float(self.rover.xmin+self.rover.xmax)/2.0-320
            if print_to_terminal:
                print("base station mean in pixels: {}".format(-x_mean))
            self.drive(0.0, (-x_mean/320)/4)
            if np.abs(x_mean) < 100:
                break

    def hauler_dump(self, value):

        """
        dump command once aligned with bin (regolith)
        """
        print("ABOUT TO DUMP")
        _cmd_pub_dump = rospy.Publisher("bin/command/position", Float64, queue_size=10)
        _cmd_message = float(value)
        _cmd_pub_dump.publish(_cmd_message)
        rospy.sleep(0.5)
        rospy.logerr("APPROACH BIN. BIN Dumping")


    def hauler_bin_reset(self, value):

        """
        dump command once aligned with bin (regolith)
        """
        _cmd_pub_dump = rospy.Publisher("bin/command/position", Float64, queue_size=10)
        _cmd_message = float(value)
        _cmd_pub_dump.publish(_cmd_message)
        rospy.sleep(0.25)
        rospy.logerr("APPROACH BIN. BIN Reset")


    def shutdown(self):
        """
        Shutdown Node
        """
        self.stop()
        rospy.logerr("APPROACH BIN. Approach bin service node is shutdown")
        rospy.sleep(1)


def main():
    try:
        rospy.init_node('approach_bin', anonymous=True)
        object_estimation_service_call = ApproachbinService()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
