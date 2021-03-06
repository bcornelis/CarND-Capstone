#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_classifier_topic import TLClassifierTopic
from light_classification.tl_classifier_opencv import TLClassifierOpenCV
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

LIGHT_CLASSIFIER_TOPIC=0
LIGHT_CLASSIFIER_TFMODELAPI=1
LIGHT_CLASSIFIER_OPENCV=2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

	self.classifier_implementation = LIGHT_CLASSIFIER_OPENCV

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
	self.listener = tf.TransformListener()
	if self.classifier_implementation == LIGHT_CLASSIFIER_OPENCV:
	    rospy.loginfo('Using OpenCV-Light-Classifier implementation')
	    self.light_classifier = TLClassifierOpenCV()
	elif self.classifier_implementation == LIGHT_CLASSIFIER_TFMODELAPI:
            rospy.loginfo('Using TensorFlow-Light-Classifier implementation')
            self.light_classifier = TLClassifier()
	else:
	    rospy.loginfo('Using Topic-Light-Classifier implementation')
            self.light_classifier = TLClassifierTopic()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # proper state?
	if (self.waypoints is None or pose is None):
	    return None

	# TODO: this should be optimized keeping the last location as a parameter
	# car position
	ref_x = pose.position.x
	ref_y = pose.position.y

	# Values we're looking for
	closest_dist = None
	closest_wp_idx = -1;

	for waypoint_idx, waypoint in enumerate(self.waypoints.waypoints):
	    # easier references
	    waypoint_x = waypoint.pose.pose.position.x
	    waypoint_y = waypoint.pose.pose.position.y

	    # calculate the distance to this waypoint
	    dist = math.sqrt((ref_x-waypoint_x)**2 + (ref_y-waypoint_y)**2)

	    if (closest_dist is None or closest_dist > dist):
		closest_dist = dist
		closest_wp_idx = waypoint_idx
	
  	return closest_wp_idx if (closest_dist is not None) else None

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image and (self.classifier_implementation == LIGHT_CLASSIFIER_TFMODELAPI or self.classifier_implementation == LIGHT_CLASSIFIER_OPENCV)):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

            #TODO find the closest visible traffic light (if one exists)
	    for stop_line_position in stop_line_positions:
		# stop line positions are (x,y) coordinates; get the closes waypoint
		stopLinePosPose = Pose()
		stopLinePosPose.position.x = stop_line_position[0]
		stopLinePosPose.position.y = stop_line_position[1]
		stopLinePosIdx = self.get_closest_waypoint(stopLinePosPose)

		# we have the index of the current stop position. If in front of the car, keep if close
		if(stopLinePosIdx > car_position ):
		    # keep if first one or closest
		    if( light is None or light_idx > stopLinePosIdx ):
			light = stopLinePosPose
			light_idx = stopLinePosIdx

        if light:
            state = self.get_light_state(light)
            return light_idx, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
