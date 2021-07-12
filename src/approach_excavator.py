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
from src2_approach_services.srv import ApproachExcavator, ApproachExcavatorResponse
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


print_to_terminal = rospy.get_param('approach_excavator/print_to_terminal', True)
ROVER_MIN_VEL = rospy.get_param('approach_excavator/rover_min_vel', 0.8)
APPROACH_TIMEOUT = rospy.get_param('approach_excavator/approach_timeout', 50)
ROTATIONAL_SPEED = rospy.get_param('approach_excavator/rotational_speed',  0.25)
# LASER_RANGE = rospy.get_param('approach_excavator/laser_range',  2.0)
LASER_RANGE = 8.0
LASER_CLOSE_RANGE = 4.5
LASER_THRES = 8.0


class Obstacle:
    """
    Obstacle Definition
    """

    def __init__(self, obstacle, distance):
        self.obstacle = obstacle
        self.distance = distance


class ApproachExcavatorService(BaseApproachClass):
    """
    Service to find the excavator and approach it using visual servoing
    """

    def __init__(self):
        """
        """
        rospy.on_shutdown(self.shutdown)
        self.rover = False
        self.robot_name = rospy.get_param('robot_name')
        print(self.robot_name)
        self.obstacles = []
        self.timeout = 60
        self.boxes = DetectedBoxes()
        rospy.sleep(2)
        rospy.loginfo("Approach Excavator service node is running")
        s = rospy.Service('approach_excavator_service', ApproachExcavator,
                          self.approach_excavator_handle)
        rospy.spin()

    def image_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        self.disparity_sub = rospy.Subscriber("disparity", DisparityImage, self.disparity_callback)
        self.image_sub = rospy.Subscriber("camera/left/image_raw", Image, self.image_callback)

        #disparity_sub = message_filters.Subscriber("disparity", DisparityImage)
        #self.ts = message_filters.ApproximateTimeSynchronizer([disparity_sub],10, 0.1, allow_headerless=True)
        #self.ts.registerCallback(self.image_callback)

    def image_unregister(self):
        rospy.loginfo("Aprroach Excavator Unregister")
        self.disparity_sub.unregister()
        self.image_sub .unregister()

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
            print("Service did not process request: " + str(exc))
        print(_find_object)
        self.boxes = _find_object.boxes
        self.check_for_obstacles(self.boxes.boxes)
        for obstacle in self.obstacle_boxes:
            dist = self.object_distance_estimation(obstacle)
            self.obstacles.append(Obstacle(obstacle,dist.object_position.point.z))

    def stop(self):
        """
        Stop the rover sending zeros cmd velocity
        """
        _cmd_publisher = rospy.Publisher("driving/cmd_vel", Twist, queue_size = 10 )
        _cmd_message = Twist()
        for i in range(1):
            _cmd_publisher.publish(_cmd_message)

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
        for i in range(1):
            _cmd_publisher.publish(_cmd_message)
            rospy.sleep(0.05)
  
    def laser_mean(self):
        """
        Return the average distance from +- 30 deg from the center of the laser
        """
        laser = rospy.wait_for_message("laser/scan", LaserScan)
        _val = 0
        _ind = 0
        for i in laser.ranges[0:99]:
            if not np.isinf(i) and i < LASER_THRES:
                _val+=i
                _ind+=1
        if _ind != 0:
            range = _val/_ind
            if print_to_terminal:
                print("Laser Range: {}".format(range))
            return range
        else:
            return 0.0

    def approach_excavator_handle(self, req):
        """
        Service for approaching the excavator in the SRC qualification
        """
        rospy.loginfo("Aprroach Excavator Service Started")
        self.image_subscriber() # start subscriber
        response = self.search_for_excavator()
        rospy.loginfo("Aprroach Excavator Service Started")
        self.image_unregister()
        return response

    def search_for_excavator(self):
        """
        Turn in place to check for excavator
        """
        _range = 0.0
        search = False
        self.command_laser(0)
        for i in range(150):
            self.turn_in_place(-1)
            self.check_for_excavator(self.boxes.boxes)
            if self.rover:
                print("Excavator found")
                if print_to_terminal:
                    print(self.rover)
                self.stop()
                self.command_laser(0.05)
                _range, search = self.approach_excavator()
                self.command_laser(0)
                if print_to_terminal:
                    print("Rover approach to excavator was: {}"
                          "and the laser distance is {}".format(search, _range))
                if search == True:
                    self.face_excavator()
                self.stop()
                break
            rospy.sleep(0.05)
        self.stop()
        response = ApproachExcavatorResponse()
        resp = Bool()
        resp.data = search
        response.success = resp
        excavator_range = Float64()
        excavator_range.data = float(_range)
        response.range = excavator_range
        if search == True:
            rospy.loginfo("Rover approached Excavator")
        else:
            rospy.logerr("Excavator was not found when running turn in place maneuver or timeout")
        self.rover = False  # reset flag variable
        return response

    def approach_excavator(self):
        """
        Visual approach the excavator,
        !!Need to improve robustness by adding some error check and obstacle avoidance
        """
        print("START approach_excavator")
        turning_offset_i = 0.0

        while rospy.get_time() == 0:
            rospy.get_time()

        init_time = rospy.get_time()

        toggle_light_ = 1

        print("ENTERING WHILE LOOP")

        while True:
            print("INSIDE WHILE LOOP")

            curr_time = rospy.get_time()
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("Timeout in approach excavator service")
                print("TIMEOUT !! in approach excavator service")
                return 0.0, False

            if toggle_light_ == 1:
                self.toggle_light(10)
                toggle_light_ = 0
            else:
                self.toggle_light(0)
                toggle_light_ = 1

            self.check_for_excavator(self.boxes.boxes)

            laser = self.laser_mean()

            print("Distance Inference")
            print(self.object_distance_estimation(self.rover).object_position.point.z)

            print("Distance Laser")
            print(laser)

            # minimum_dist = 10
            # if laser < 5.0:
            #     minimum_dist = 3.0
            # if laser < 10.0:
            #     minimum_dist = ROVER_MIN_VEL*10
            # speed = minimum_dist/10.0  


            speed = ROVER_MIN_VEL

            x_mean_base = float(self.rover.xmin+self.rover.xmax)/2.0-320
            turning_offset = 0.0
            rotation_speed = (-x_mean_base/840+turning_offset+0.5*turning_offset_i)

            if laser < LASER_CLOSE_RANGE and laser != 0.0:
                print("Approached to target Laser Range. Stopping.")
                self.stop()
                break
            elif laser < LASER_RANGE:
                print("Close from target. Drive.")
                self.check_for_excavator(self.boxes.boxes)
                x_mean = float(self.rover.xmin+self.rover.xmax)/2.0-320
                if np.abs(x_mean) >= 200:
                    self.stop()
                    self.face_excavator()
                else:
                    print("Far from target. Drive.")
                    print("Base speed : ", speed/4)
                    print("Base rotational speed : ", rotation_speed/4)
                    self.drive(speed/4, rotation_speed/4)
                    # if within LASER_RANGE, approach to half that distance ~2m? very slowly
            else:
                print("Far from target. Drive.")
                print("Base speed : ", speed)
                print("Base rotational speed : ", rotation_speed)
                self.drive(speed, rotation_speed)
                # (self.rover.xmax-self.rover.xmin) > 200

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

            # print("SPEED : ", speed)
            # rotation_speed = -x_mean_base/840+turning_offset+0.5*turning_offset_i
            # print("ENTERING DRIVE")
            # self.drive(speed, rotation_speed)
            # print("Distance Inference")
            # print(self.object_distance_estimation(self.rover).object_position.point.z)
            # print("Distance Laser")
            # print(laser)
            # # (self.rover.xmax-self.rover.xmin) > 200
            # if laser < LASER_RANGE and laser != 0.0:
            #     self.stop()
            #     self.face_excavator()
            #     # if within LASER_RANGE, approach to half that distance ~2m? very slowly
            #     print("******DOING SLOW/CLOSE EXCAVATOR APPROACH********")
            #     self.drive(speed/4, 0.0)
            #     if laser < LASER_RANGE/2 and laser !=0.0:
            #         print("+++++++++++++++DID CLOSE APPROACH AND IS WITHIN 2.25m")
            #         self.stop()
            #         break
                
        print("Close to Excavator")
        self.stop()
        return self.laser_mean(), True

    def check_for_excavator(self, boxes):
        """
        Check if excavator exist in the bounding boxes
        """
        for box in boxes:
            if box.id == 3:
                self.rover = box

    def face_excavator(self):
        """
        Service to align the rover to the excavator using
        bounding boxes from inference node
        """
        while True:
            print("Trying to face excavator")
            self.check_for_excavator(self.boxes.boxes)
            x_mean = float(self.rover.xmin+self.rover.xmax)/2.0-320
            if print_to_terminal:
                print("base station mean in pixels: {}".format(-x_mean))
            self.drive(0.0, (-x_mean/320)/4)
            if np.abs(x_mean) < 200:
                break


    def shutdown(self):
        """
        Shutdown Node
        """
        self.stop()
        rospy.loginfo("Approach excavator service node is shutdown")
        rospy.sleep(1)


def main():
    try:
        rospy.init_node('approach_excavator', anonymous=True)
        object_estimation_service_call = ApproachExcavatorService()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
