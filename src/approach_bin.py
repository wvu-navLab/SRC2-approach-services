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
from src2_object_detection.srv import ApproachBin, ApproachBinResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from srcp2_msgs.srv import LocalizationSrv, SpotLightSrv  # AprioriLocationSrv,

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
LASER_RANGE = 4.5
ROTATIONAL_SPEED = rospy.get_param('approach_bin/rotational_speed',  0.25)


class Obstacle:
    """
    Obstacle Definition
    """

    def __init__(self, obstacle, distance):
        self.obstacle = obstacle
        self.distance = distance


class ApproachbinService:
    """
    Service to find the bin and approach it using visual servoing
    """

    def __init__(self):
        """
        """
        rospy.on_shutdown(self.shutdown)
        self.rover = False
        self.obstacles = []
        self.timeout = 60
        self.stereo_subscriber()
        rospy.sleep(2)
        rospy.loginfo("Approach bin service node is running")
        s = rospy.Service('approach_bin_service', ApproachBin,
                          self.approach_bin_handle)
        rospy.spin()

    def stereo_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        disparity_sub = message_filters.Subscriber("disparity", DisparityImage)
        boxes_sub = message_filters.Subscriber("DetectedBoxes", DetectedBoxes)
        ts = message_filters.ApproximateTimeSynchronizer(
            [disparity_sub, boxes_sub], 10, 0.1, allow_headerless=True)

        ts.registerCallback(self.image_callback)

    def image_callback(self, disparity, boxes):
        """
        Subscriber callback for the stereo camera, with synchronized images
        """
        self.obstacles = []
        self.disparity = disparity
        self.boxes = boxes
        self.check_for_obstacles(self.boxes.boxes)
        for obstacle in self.obstacle_boxes:
            dist = self.object_distance_estimation(obstacle)
            self.obstacles.append(Obstacle(obstacle, dist.object_position.point.z))

    def approach_bin_handle(self, req):
        """
        Service for approaching the bin in the SRC qualification
        """
        response = self.search_for_bin()
        rospy.loginfo("Aprroach bin Service Started")
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
                print("bin found")
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
            rospy.loginfo("Rover approached bin")
        else:
            rospy.logerr("bin was not found when running turn in place maneuver or timeout")
        self.rover = False  # reset flag variable
        return response

    def approach_bin(self):
        """
        Visual approach the bin,
        !!Need to improve robustness by adding some error check and obstacle avoidance
        """
        print("START approach_bin")
        turning_offset_i = 0.0
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()
        toggle_light_ = 1
        print("ENTERING WHILE LOOP")
        while True:
            print("INSIDE WHILE LOOP")
            self.check_for_bin(self.boxes.boxes)
            x_mean_base = float(self.rover.xmin+self.rover.xmax)/2.0-320
            minimum_dist = 10
            turning_offset = 0.0
            laser = self.laser_mean()
            print("LASER : ", laser)
            if laser < 10.0:
                minimum_dist = ROVER_MIN_VEL*10
            if toggle_light_ == 1:
                self.toggle_light(10)
                toggle_light_ = 0
            else:
                self.toggle_light(0)
                toggle_light_ = 1
            curr_time = rospy.get_time()
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("Timeout in approach bin service")
                print("TIMEOUT !! in approach bin service")
                return 0.0, False
            # for obstacle_ in self.obstacles:
            #     print("ENTERING OBSTACLE LOOP")
            #     obstacle_mean_ = float(obstacle_.obstacle.xmin+obstacle_.obstacle.xmax)/2.0-320
            #     turning_offset_i = turning_offset
            #     if obstacle_.distance > 0.1:
            #         if obstacle_.distance < minimum_dist:
            #             minimum_dist = obstacle_.distance
            #         if obstacle_.distance < 8:
            #             turning_offset += np.sign(obstacle_mean_)*0.3 * \
            #                 (1-np.abs(obstacle_mean_)/320.0)
            # print("EXITING OBSTACLE LOOP")
            if laser < 5:
                minimum_dist = 3.0
            speed = minimum_dist/10.0
            print("SPEED : ", speed)
            rotation_speed = -x_mean_base/840+turning_offset+0.5*turning_offset_i
            print("ENTERING DRIVE")
            self.drive(speed, rotation_speed)
            print("Distance Inference")
            print(self.object_distance_estimation(self.rover).object_position.point.z)
            print("Distance Laser")
            print(laser)
            # (self.rover.xmax-self.rover.xmin) > 200
            if laser < LASER_RANGE and laser != 0.0:
                break
        print("Close to bin")
        print("BEGIN DUMPING")
        # self.hauler_dump(2.0)

        self.stop()
        return self.laser_mean(), True

    def turn_in_place(self, direction):
        """
        Turn in place clockwise or counter clockwise
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size=10)
        _cmd_message = Twist()
        _cmd_message.angular.z = ROTATIONAL_SPEED*direction
        for i in range(2):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

    def object_distance_estimation(self, object):
        """
        Estimate distance from arg object to the camera of the robot.
        Requires disparity image
        """
        rospy.wait_for_service('object_estimation')  # Change the name of the service
        object_estimation_call = rospy.ServiceProxy('object_estimation', ObjectEstimation)
        try:
            object_distance = object_estimation_call(object, self.disparity)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return(object_distance)

    def drive(self, speed, heading):
        """
        Drive function, send args to the cmd velocity topic
        Args:
        speed: linear x velocity of the robot
        heading: angular z velocity of the robot
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size=10)
        _cmd_message = Twist()
        _cmd_message.linear.x = speed
        _cmd_message.angular.z = heading
        for i in range(5):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)

    def stop(self):
        """
        Stop the rover sending zeros cmd velocity
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size=1)
        _cmd_message = Twist()
        _cmd_publisher.publish(_cmd_message)
        _cmd_publisher.publish(_cmd_message)

    def check_for_bin(self, boxes):
        """
        Check if bin exist in the bounding boxes
        """
        for box in boxes:
            if box.id == 6: # bin id == 6 -- regolith id == 12
                self.rover = box


    def check_for_regolith(self, boxes):
        """
        check for grey regolith above bin to perform alignment (face_regolith)
        """
        for box in boxes:
            if box.id == 12: # bin id == 6 -- regolith id == 12
                self.rover = box

    def check_for_obstacles(self, boxes):
        """
        Check if obstacles exist in the bounding boxes
        """
        self.obstacle_boxes = []
        for box in boxes:
            if box.id == 5:
                self.obstacle_boxes.append(box)

    def laser_mean(self):
        """
        Return the average distance from +- 30 deg from the center of the laser
        """
        laser = rospy.wait_for_message("laser/scan", LaserScan)
        _val = 0
        _ind = 0
        for i in laser.ranges[20:80]:
            if not np.isinf(i):
                _val += i
                _ind += 1
        if _ind != 0:
            range = _val/_ind
            if print_to_terminal:
                print("Laser Range: {}".format(range))
            return range
        else:
            return 0.0

    def face_regolith(self):
        """
        Service to align the rover to the bin using
        bounding boxes from inference node
        """
        while True:
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
        for i in range(5):
            _cmd_pub_dump.publish(_cmd_message)
            rospy.sleep(0.05)
        print("*******DUMP FINISHED")
        self.hauler_bin_reset(0.0)


    def hauler_bin_reset(self, value):

        """
        dump command once aligned with bin (regolith)
        """
        _cmd_pub_dump = rospy.Publisher("bin/command/position", Float64, queue_size=10)
        _cmd_message = float(value)
        for i in range(5):
            _cmd_pub_dump.publish(_cmd_message)
            rospy.sleep(0.05)
        print("*******BIN RESET")



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
        rospy.loginfo("Approach bin service node is shutdown")
        rospy.sleep(1)


def main():
    try:
        rospy.init_node('approach_bin', anonymous=True)
        object_estimation_service_call = ApproachbinService()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
