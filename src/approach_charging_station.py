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
from src2_object_detection.srv import DistanceEstimation, DistanceEstimationResponse
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
from base_approach_class import BaseApproachClass

ROVER_MIN_VEL = rospy.get_param('approach_base_station_service/rover_min_vel', 0.8)
APPROACH_TIMEOUT = rospy.get_param('approach_base_station_service/approach_timeout', 50)
LASER_RANGE = 5

class Obstacle:
    """
    Obstacle Definition
    """
    def __init__(self,obstacle, distance):
        self.obstacle = obstacle
        self.distance = distance


class ApproachChargingStationService(BaseApproachClass):
    """
    Service to find the charging station and approach it using visual servoing
    """
    def __init__(self):
        """
        """
        self.robot_name = rospy.get_param("robot_name")
        rospy.loginfo("[{}] Approach Charging Station service node is running".format(self.robot_name))
        rospy.on_shutdown(self.shutdown)
        self.base = False
        self.obstacles = []
        self.timeout = 60
        self.boxes = DetectedBoxes()
        self.mast_camera_publisher_pitch = rospy.Publisher("sensor/pitch/command/position", Float64, queue_size = 10 )
        rospy.sleep(2)
        s = rospy.Service('approach_charging_station_service', ApproachChargingStation, self.approach_charging_station_handle)
        rospy.spin()

    def image_subscriber(self):
        """
        Define the Subscriber for disparity image
        """
        self.disparity_sub = rospy.Subscriber("disparity", DisparityImage, self.image_callback)

    def image_unregister(self):
        """
        Unregister the disparity image subscriber
        """
        self.disparity_sub.unregister()

    def image_callback(self, disparity):
        """
        callback for the disparity image and for calling the inference service
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
        self.boxes = _find_object.boxes
        self.check_for_obstacles(self.boxes.boxes)
        for obstacle in self.obstacle_boxes:
            dist = self.object_distance_estimation(obstacle)
            self.obstacles.append(Obstacle(obstacle,dist.object_position.point.z))

    def approach_charging_station_handle(self, req):
        """
        Service for approaching the base station in the SRC qualification
        """
        rospy.loginfo("[{}] Approach Base Station Service Started".format(self.robot_name))
        self.image_subscriber()
        rospy.sleep(0.5)
        response = self.search_for_base_station()
        #subscriber unregister
        self.image_unregister()
        self.toggle_light(20) #turn on the lights at the end
        print("Subscriber unregisterd")
        return response

    def search_for_base_station(self):
        """
        Turn in place to check for base station
        """
        self.mast_camera_publisher_pitch.publish(0.0)
        _range = 0.0
        search = False
        self.check_for_base_station(self.boxes.boxes)
        toggle_light_ = 1
        double_check = False
        self.toggle_light(10)
        if self.base: #try in front
            rospy.loginfo("[{}] Found the charging station found first time".format(self.robot_name))
            self.base = False
            self.stop()
            rospy.sleep(0.8)
            self.check_for_base_station(self.boxes.boxes)
            if self.base:
                rospy.loginfo("[{}] Found the charging station found second time".format(self.robot_name))
                double_check == True
                if print_to_terminal:
                    print(self.base)
                self.stop()
                _range, search = self.approach_base_station()
                if search == True:
                    self.face_base()
                self.stop()
        if double_check == False:        #else turn in place
            for i in range(240):
                # Toggle light
                if toggle_light_ == 1:
                    self.toggle_light(10)
                    toggle_light_ = 0
                else:
                    self.toggle_light(0)
                    toggle_light_ = 1
                #turn in place
                self.turn_in_place(1)
                self.check_for_base_station(self.boxes.boxes)
                if self.base: #try in front
                    rospy.loginfo("[{}] Found the charging station found first time".format(self.robot_name))
                    self.base = False
                    self.stop()
                    rospy.sleep(0.5)
                    self.check_for_base_station(self.boxes.boxes)
                    if self.base:
                        rospy.loginfo("[{}] Found the charging station found second time".format(self.robot_name))
                        self.stop()
                        _range, search = self.approach_base_station()
                        if search == True:
                            self.face_base()
                        self.stop()
                        break
        # Reset camera pitch to zero
        self.mast_camera_publisher_pitch.publish(0.0)
        self.stop()

        response = ApproachChargingStationResponse()
        resp = Bool()
        resp.data = search
        response.success = resp
        base_station_range = Float64()
        base_station_range.data = float(_range)
        response.range = base_station_range
        if search == True:
            rospy.loginfo("[{}] Rover approached base station - Success".format(self.robot_name))
        else:
            rospy.logerr("[{}] Base Station was not found when running turn in place maneuver or timeout - Failure".format(self.robot_name))
        self.base = False # reset flag variable
        return response

    def approach_base_station(self):
        """
        Visual approach the base station,
        !!Need to improve robustness by adding some error check and obstacle avoidance
        """
        #Force init time at zero for watchdog timer
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()
        #Temporary variables
        toggle_light_ = 1
        while True:
            # Timeout for failing and leaving the loop
            curr_time = rospy.get_time()
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("[{}] Timeout in approach charging station service".format(self.robot_name))
                return 0.0, False
            # Toggle rover lights
            if toggle_light_ == 1:
                self.toggle_light(10.0)
                toggle_light_ = 0
            else:
                self.toggle_light(0.0)
                toggle_light_ = 1
            #Check for base station
            self.check_for_base_station(self.boxes.boxes)
            # Check for Rover as obstacle and stop
            self.check_for_rover(self.boxes.boxes)
            if self.rover_boxes:
                for object_ in self.rover_boxes:
                    dist = self.object_distance_estimation(object_).object_position.point.z
                    self.check_for_rover(self.boxes.boxes)
                    if dist < 5.0 and dist != 0.0:
                        rospy.logwarn("[{}] There's a rover in front of me. Distance: {}".format(self.robot_name, dist))
                        self.stop()
                        rospy.sleep(10)
            # Calculate base station distance from different sensors
            median_distance = self.base_point_estimation().point.z
            rospy.loginfo("[{}] Charging station distance from disparity image: {}".format(self.robot_name, median_distance))
            laser = self.laser_mean()
            rospy.loginfo("Charging station distance from laser mean distance: {}".format(self.robot_name, laser))
            # logic to stop the rover when close to the base station
            if median_distance == 0.0 or laser == Inf:
                distance = Inf
            elif median_distance > 1.5*laser:
                distance = median_distance
            else:
                distance = laser
            if distance < LASER_RANGE:
                break
            else:
                x_mean_base = float(self.base.xmin+self.base.xmax)/2.0-320
                speed = min(ROVER_MIN_VEL*(distance-5.0)/10.0 + ROVER_MIN_VEL/4, ROVER_MIN_VEL)
                rotation_speed = -x_mean_base/840
                self.drive(speed, rotation_speed)
        rospy.loginfo("[{}] approached charging station - Close enough ".format(self.robot_name))
        self.stop()
        return self.laser_mean(), True


    def face_base(self):
        """
        Service to align the rover to the base station using
        bounding boxes from inference node
        """
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()

        while True:
            #Time out
            curr_time = rospy.get_time()
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("[{}] Timeout in align to charging station".format(self.robot_name))
                break
            # Check for base station
            self.check_for_base_station(self.boxes.boxes)
            # Get mean point
            x_mean = float(self.base.xmin+self.base.xmax)/2.0-320
            # Turn in place to face mean
            self.drive(0.0, (-x_mean/320)/4) # was 640
            if np.abs(x_mean)<40: # was 10
                break

    def base_point_estimation(self):
        """
        Estimate the median distance of the charging station from a reduced bounding box
        Bounding box redeuced in 64% area
        """
        self.check_for_base_station(self.boxes.boxes)
        smaller_bounding_box = self.base
        smaller_bounding_box.xmin = int(self.base.xmin + (self.base.xmax - self.base.xmin)*0.2)
        smaller_bounding_box.xmax = int(self.base.xmax - (self.base.xmax - self.base.xmin)*0.2)
        smaller_bounding_box.ymin = int(self.base.ymin + (self.base.ymax - self.base.ymin)*0.2)
        smaller_bounding_box.ymax = int(self.base.ymax - (self.base.ymax - self.base.ymin)*0.2)
        object_point = self.distance_estimation(smaller_bounding_box)
        _point = object_point.point
        return(_point)

    def distance_estimation(self, object):
        """
        Estimate distance from arg object to the camera of the object.
        Requires disparity image
        """
        rospy.wait_for_service('distance_estimation') #Change the name of the service
        distance_estimation_call = rospy.ServiceProxy('distance_estimation', DistanceEstimation)
        try:
            object_distance = distance_estimation_call(object, self.disparity)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return(object_distance)


    def shutdown(self):
        """
        Shutdown Node
        """
        self.stop()
        rospy.loginfo("[{}] Approach charging station service node is shutdown".format(self.robot_name))
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('approach_base_station',anonymous=True)
        object_estimation_service_call = ApproachChargingStationService()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
