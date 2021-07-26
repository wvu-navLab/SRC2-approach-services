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
from src2_approach_services.srv import FindBin, FindBinResponse
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from srcp2_msgs.srv import LocalizationSrv, SpotLightSrv  # AprioriLocationSrv,
from base_approach_class import BaseApproachClass
from nav_msgs.msg import Odometry
import numpy as np
import tf.transformations as t_


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


class FindBinService(BaseApproachClass):
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
        self.window = False
        self.timeout = 60
        self.boxes = DetectedBoxes()
        self.mast_camera_publisher_pitch = rospy.Publisher("sensor/pitch/command/position", Float64, queue_size = 10 )
        rospy.sleep(2)
        rospy.loginfo("[{}] Find bin service node is running ".format(self.robot_name))
        s = rospy.Service('find_bin_service', FindBin,
                          self.find_bin_handle)
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

    def find_bin_handle(self, req):
        """
        Service for finding the bin
        """
        self.image_subscriber()
        self.toggle_light(0) #turn off lights at the beginning
        rospy.loginfo("[{}] Find Bin Service Started".format(self.robot_name))
        response = self.search_for_bin()
        self.image_unregister()
        self.toggle_light(20) #turn lights at the end
        return response

    def search_for_bin(self):
        """
        Maneuvers for finding the bin in the image - turn in place
        """
        self.hauler_pose = Pose()
        status = True
        x = 0.0
        y = 0.0
        self.mast_camera_publisher_pitch.publish(0.0)
        for i in range(360):
            self.turn_in_place(-0.90)
            self.check_for_bin(self.boxes.boxes)
            if self.bin:
                rospy.loginfo("[{}] Bin was found with confidene of {}".format(self.robot_name,self.bin.confidence))
                self.stop()
                status, x, y = self.detect_initial_position_and_adjust()
                break
        self.stop()
        response = FindBinResponse()
        resp = Bool()
        resp.data = status
        response.success = resp
        response.x = Float64(x)
        response.y = Float64(y)
        self.bin = False  # reset flag variable
        self.mast_camera_publisher_pitch.publish(0.0)
        print("Old Pose: {}, {}".format(self.hauler_pose.position.x, self.hauler_pose.position.y))
        print("New Pose: {}, {}".format(x,y))
        return response


    def detect_initial_position_and_adjust(self):
        # Add capability of adjusting the parking position if parked far from processing plant
        self.face_window()
        hauler_odom = rospy.wait_for_message("localization/odometry/sensor_fusion", Odometry)
        self.hauler_pose = hauler_odom.pose.pose
        hauler_orientation_euler = t_.euler_from_quaternion([self.hauler_pose.orientation.x,
                                                        self.hauler_pose.orientation.y,
                                                        self.hauler_pose.orientation.z,
                                                        self.hauler_pose.orientation.w])

        print("ORIENTATION: {}".format(hauler_orientation_euler))
        print("ORIENTATION: {}".format(hauler_orientation_euler))
        print("ORIENTATION: {}".format(hauler_orientation_euler))
        _angle = np.rad2deg(hauler_orientation_euler[2])
        print(_angle)
        print(_angle%360)

        #(alpha - lower) % 360 <= (upper - lower) % 360
        # (angle-lower_bound)%360 <= (upper_pount - lower_bound) %360
        if (_angle - 135) % 360 <= (90) % 360:
            print("WITHIN BOUNDARY")
            return True , 0.0, 0.0
        else:
            print("OUTSIDE BOUNDARY")
            theta = (hauler_orientation_euler[2])
            print("Angle")
            print(np.rad2deg(theta))
            dist = self.object_distance_estimation(self.bin).object_position.point.z
            print("Dist: {}".format(dist))
            if dist > 10.0:
                dist = 10.0
            _x = 10 - dist*np.cos(theta)
            _y =  dist*np.sin(theta)
            print("X = {}".format(_x))
            print("Y = {}".format(_y))
            x = _x + self.hauler_pose.position.x
            y = _y + self.hauler_pose.position.y

            return False, x, y

    def face_window(self):
        """
        Service to align the rover to the bin using
        bounding boxes from inference node
        """
        # Initialize timeout watchdog:
        while rospy.get_time() == 0:
            rospy.get_time()
        init_time = rospy.get_time()
        rospy.loginfo("[{}] Approach bin trying to face excavator".format(self.robot_name))
        while True:
            curr_time = rospy.get_time() # Timeout break:
            if curr_time - init_time > APPROACH_TIMEOUT:
                rospy.logerr("[{}] Timeout in approach bin service (Face window)".format(self.robot_name))
                break
            # get the latest processing plant bin location from bounding boxes
            self.check_for_bin(self.boxes.boxes)
            x_mean = float(self.bin.xmin+self.bin.xmax)/2.0-320
            self.drive(0.0, (-x_mean/320)/4)
            if np.abs(x_mean) < 100:
                self.stop()
                break


    def shutdown(self):
        """
        Shutdown Node
        """
        self.stop()
        rospy.loginfo("[{}] Find bin service node is shutdown".format(self.robot_name))
        rospy.sleep(1)

def main():
    try:
        rospy.init_node('find_bin', anonymous=True)
        find_bin_service = FindBinService()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
