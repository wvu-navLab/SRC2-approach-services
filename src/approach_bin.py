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
from src2_object_detection.msg import Box
from src2_object_detection.msg import DetectedBoxes
from src2_object_detection.srv import DistanceEstimation, DistanceEstimationResponse
from src2_object_detection.srv import ObjectEstimation, ObjectEstimationResponse
from src2_object_detection.srv import FindObject, FindObjectResponse
from src2_approach_services.srv import ApproachBin, ApproachBinResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from srcp2_msgs.srv import LocalizationSrv, SpotLightSrv  # AprioriLocationSrv,
from base_approach_class import BaseApproachClass
import numpy as np

ROVER_MIN_VEL = rospy.get_param('approach_bin/rover_min_vel', 0.8)
APPROACH_TIMEOUT = rospy.get_param('approach_bin/approach_timeout', 50)
LASER_RANGE = 5.50
LASER_CLOSE_RANGE = 2.75


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
        Initialize service server and initialize variables and params
        """
        self.robot_name = rospy.get_param("robot_name")
        rospy.on_shutdown(self.shutdown)
        self.bin = False
        self.obstacles = []
        self.timeout = 60
        self.boxes = DetectedBoxes()
        self.mast_camera_publisher_pitch = rospy.Publisher("sensor/pitch/command/position", Float64, queue_size = 10 )
        rospy.sleep(2)
        rospy.loginfo("[{}] Approach bin service node is running ".format(self.robot_name))
        s = rospy.Service('approach_bin_service', ApproachBin,
                          self.approach_bin_handle)
        rospy.spin()

    def image_subscriber(self):
        """
        Define the Image Subscriber
        """
        self.disparity_sub = rospy.Subscriber("disparity", DisparityImage, self.disparity_callback)

    def image_unregister(self):
        """
        Method for unregister the image subscriber
        """
        rospy.loginfo("Approach Bin Unregister Images for {}".format(self.robot_name))
        self.disparity_sub.unregister()

    def disparity_callback(self, disparity):
        """
        Subscriber callback for disparity image and method for calling the service
        to find detected boxes with inference. Process both and update obstacles list
        """
        self.obstacles = []
        self.disparity = disparity
        ##Calling the service
        _find_object = FindObjectResponse()
        rospy.wait_for_service('/find_object')
        find_object =rospy.ServiceProxy('/find_object', FindObject)
        try:
            _find_object = find_object(robot_name = self.robot_name)
        except rospy.ServiceException as exc:
            rospy.logerr("Approach Bin Service did not process request: " + str(exc))
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
        self.toggle_light(0) #turn off lights at the beginning
        response = self.search_for_bin()
        rospy.loginfo("[{}] Approach Bin Service Started".format(self.robot_name))
        self.image_unregister()
        self.toggle_light(20) #turn lights at the end
        return response

    def search_for_bin(self):
        """
        Maneuvers for finding the bin in the image - turn in place
        """
        self.mast_camera_publisher_pitch.publish(0.0)
        _range = 0.0
        search = False
        for i in range(240):
            self.turn_in_place(-1)
            self.check_for_bin(self.boxes.boxes)
            if self.bin:
                rospy.loginfo("[{}] Bin was found with confidene of {}".format(self.robot_name,self.bin.confidence))
                self.stop()
                _range, search = self.approach_bin()
                if search == True:
                    self.face_regolith()
                self.stop()
                break
        self.stop()
        response = ApproachBinResponse()
        resp = Bool()
        resp.data = search
        response.success = resp
        bin_range = Float64()
        bin_range.data = float(_range)
        response.range = bin_range
        if search == True:
            rospy.loginfo("[{}] Rover approached bin - Success".format(self.robot_name))
        else:
            rospy.logerr("[{}] Bin was not found when running turn in place maneuver or timeout - Failed approach".format(self.robot_name))
        self.bin = False  # reset flag variable
        self.mast_camera_publisher_pitch.publish(0.0)
        return response

    def approach_bin(self):
        """
        Visual approach the bin, with laser feedback
        """
        rospy.loginfo("[{}] Driving maneuver for approach bin started".format(self.robot_name))
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()
        toggle_light_ = 1
        while True:
            # Timeout check
            curr_time = rospy.get_time()
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("[{}] Timeout in approach bin service".format(self.robot_name))
                return 0.0, False
            # Toggle the rover lights for better inference
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
                    if dist < 7.0 and dist != 0.0:
                        rospy.logwarn("[{}] There's a rover in front of me. Distance: {}".format(self.robot_name,dist))
                        self.stop()
                        rospy.sleep(10)
            laser = self.laser_mean()
            rospy.loginfo("[{}] Current laser to the bin: {}".format(self.robot_name,laser))
            speed = ROVER_MIN_VEL
            x_mean_base = float(self.bin.xmin+self.bin.xmax)/2.0-320
            rotation_speed = -x_mean_base/840
            # Compare the distances to decide what to do
            if laser < LASER_CLOSE_RANGE:
                rospy.loginfo("[{}] Approached target, distance was: {}. Stopping.".format(self.robot_name,laser))
                self.stop()
                break
            elif laser < LASER_RANGE:
                self.check_for_bin(self.boxes.boxes)
                x_mean = float(self.bin.xmin+self.bin.xmax)/2.0-320
                if np.abs(x_mean) >= 100:
                    self.stop()
                    self.face_regolith()
                else:
                    self.drive(speed/2, rotation_speed/4)
                    # if within LASER_RANGE, approach to half that distance ~2m? very slowly
            else:
                self.drive(speed, rotation_speed)
        # Call Bin dumping for the hauler
        rospy.sleep(0.1)
        rospy.loginfo("[{}] Approach bin - Beginning first dump maneuver.".format(self.robot_name))
        self.drive(speed/8, 0.0)
        rospy.sleep(5)
        self.hauler_dump(1.57)
        rospy.sleep(10)
        self.hauler_dump(3.14)
        self.stop()
        self.hauler_bin_reset(0.0)
        self.stop()
        # move 10cm back and do it again
        rospy.loginfo("[{}] Approach bin - Beginning second dump maneuver.".format(self.robot_name))
        self.drive(-speed/8, 0.0)
        rospy.sleep(1)
        self.stop()
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
        # Initialize timeout watchdog:
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()
        while True:
            curr_time = rospy.get_time() # Timeout break:
            rospy.loginfo("[{}] Approach bin trying to face excavator".format(self.robot_name))
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("[{}] Timeout in approach bin service (Face Regolith)".format(self.robot_name))
                break
            # get the latest processing plant bin location from bounding boxes
            self.check_for_bin(self.boxes.boxes)
            x_mean = float(self.bin.xmin+self.bin.xmax)/2.0-320
            self.drive(0.0, (-x_mean/320)/4)
            if np.abs(x_mean) < 100:
                break

    def hauler_dump(self, value):
        """
        Dump command once aligned with bin (regolith)
        """
        rospy.loginfo("[{}] Dumping - angle: {}".format(self.robot_name,value))
        _cmd_pub_dump = rospy.Publisher("bin/command/position", Float64, queue_size=10)
        _cmd_message = float(value)
        _cmd_pub_dump.publish(_cmd_message)
        rospy.sleep(0.5)

    def hauler_bin_reset(self, value):
        """
        Dump command once aligned with bin (regolith)
        """
        _cmd_pub_dump = rospy.Publisher("bin/command/position", Float64, queue_size=10)
        _cmd_message = float(value)
        _cmd_pub_dump.publish(_cmd_message)
        rospy.sleep(0.25)
        rospy.loginfo("[{}] Dumping - bin reset".format(self.robot_name))

    def shutdown(self):
        """
        Shutdown Node
        """
        self.stop()
        rospy.loginfo("[{}] Approach bin service node is shutdown".format(self.robot_name))
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('approach_bin', anonymous=True)
        object_estimation_service_call = ApproachbinService()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
