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

print_to_terminal = rospy.get_param('approach_base_station_service/print_to_terminal', False)
ROVER_MIN_VEL = rospy.get_param('approach_base_station_service/rover_min_vel', 0.8)
APPROACH_TIMEOUT = rospy.get_param('approach_base_station_service/approach_timeout', 50)
LASER_RANGE = 9

class Obstacle:
    """
    Obstacle Definition
    """
    def __init__(self,obstacle, distance):
        self.obstacle = obstacle
        self.distance = distance


class ApproachChargingStationService(BaseApproachClass):
    """
    Service to find the base station and approach it using visual servoing
    """
    def __init__(self):
        """
        """
        # self.robot_name = rospy.get_param('~robot_name')
        self.robot_name = 'small_scout_1' 
        rospy.on_shutdown(self.shutdown)
        self.base = False
        self.obstacles = []
        self.timeout = 60
        rospy.sleep(2)
        rospy.loginfo("Approach Base Station service node is running")
        s = rospy.Service('approach_charging_station_service', ApproachChargingStation, self.approach_charging_station_handle)
        rospy.spin()

    def image_subscriber(self):
        """
        Define the Subscriber with time synchronization among the image topics
        from the stereo camera
        """
        self.disparity_sub = rospy.Subscriber("disparity", DisparityImage, self.image_callback)
        #disparity_sub = message_filters.Subscriber("disparity", DisparityImage)
        #self.ts = message_filters.ApproximateTimeSynchronizer([disparity_sub],10, 0.1, allow_headerless=True)
        #self.ts.registerCallback(self.image_callback)

    def image_unregister(self):
        self.disparity_sub.unregister()


    def image_callback(self, disparity):
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
        self.boxes = _find_object.boxes
        self.check_for_obstacles(self.boxes.boxes)
        for obstacle in self.obstacle_boxes:
            dist = self.object_distance_estimation(obstacle)
            self.obstacles.append(Obstacle(obstacle,dist.object_position.point.z))

    def approach_charging_station_handle(self, req):
        """
        Service for approaching the base station in the SRC qualification
        """
        rospy.loginfo("Aprroach Base Station Service Started")
        self.image_subscriber()
        rospy.sleep(0.5) #fix it - subscriber needs to run one time before the rest of the code starts
        response = self.search_for_base_station()

        #subscriber unregister #todo figure out how to unregisterd
        self.image_unregister()
        print("Subscriber unregisterd")
        return response

    def search_for_base_station(self):
        """
        Turn in place to check for base station
        """
        search = False
        _range = 0.0
        for i in range(150):
            self.turn_in_place(-1)
            self.check_for_base_station(self.boxes.boxes)
            if self.base:
                print("Base Station found")
                if print_to_terminal:
                    print(self.base)
                self.stop()
                _range, search = self.approach_base_station()
                if print_to_terminal:
                    print("Rover approach to base station was: {}"
                        "and the laser distance is {}".format(search, _range))
                if search == True:
                    self.face_base()
                self.stop()
                break
        response = ApproachChargingStationResponse()
        resp = Bool()
        resp.data = search
        response.success = resp
        base_station_range = Float64()
        base_station_range.data = float(_range)
        response.range = base_station_range
        if search == True:
            rospy.loginfo("Rover approached base station")
        else:
            rospy.logerr("Base Station was not found when running turn in place maneuver or timeout")
        self.base = False # reset flag variable
        return response

    def approach_base_station(self):
        """
        Visual approach the base station,
        !!Need to improve robustness by adding some error check and obstacle avoidance
        """
        turning_offset_i = 0.0
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()
        toggle_light_ = 1

        while True:
            self.check_for_base_station(self.boxes.boxes)
            x_mean_base = float(self.base.xmin+self.base.xmax)/2.0-320
            minimum_dist = 10
            turning_offset = 0.0
            laser = self.laser_mean()
            if laser <10.0:
                minimum_dist = ROVER_MIN_VEL*10
            if toggle_light_ == 1:
                self.toggle_light(10.0)
                toggle_light_ = 0
            else:
                self.toggle_light(0.0)
                toggle_light_ = 1
            curr_time = rospy.get_time()
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("Timeout in approach base station service")
                return 0.0, False
            for obstacle_ in self.obstacles:
                obstacle_mean_ = float(obstacle_.obstacle.xmin+obstacle_.obstacle.xmax)/2.0-320
                turning_offset_i = turning_offset
                if obstacle_.distance > 0.1:
                    if obstacle_.distance < minimum_dist:
                        minimum_dist = obstacle_.distance
                    if obstacle_.distance < 8:
                        turning_offset += np.sign(obstacle_mean_)*0.3*(1-np.abs(obstacle_mean_)/320.0)
            speed = minimum_dist/10.0
            rotation_speed = -x_mean_base/840+turning_offset+0.5*turning_offset_i
            self.drive(speed, rotation_speed)
            if laser < LASER_RANGE and laser!=0.0: #(self.base.xmax-self.base.xmin) > 340
                break
        print("Close to base station")
        self.stop()
        return self.laser_mean(), True

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


    def check_for_base_station(self,boxes):
        """
        Check if base station exist in the bounding boxes
        """
        for box in boxes:
            if box.id == 1:
                self.base = box

    def check_for_obstacles(self,boxes):
        """
        Check if obstacles exist in the bounding boxes
        """
        self.obstacle_boxes = []
        for box in boxes:
            if box.id == 5:
                self.obstacle_boxes.append(box)



    def face_base(self):
        """
        Service to align the rover to the base station using
        bounding boxes from inference node
        """
        while True:
            self.check_for_base_station(self.boxes.boxes)
            x_mean = float(self.base.xmin+self.base.xmax)/2.0-320
            if print_to_terminal:
                print("base station mean in pixels: {}".format(-x_mean))
            self.drive(0.0, (-x_mean/320)/4) # was 640
            if np.abs(x_mean)<40: # was 10
                break

def main():
    try:
        rospy.init_node('approach_base_station',anonymous=True)
        object_estimation_service_call = ApproachChargingStationService()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
