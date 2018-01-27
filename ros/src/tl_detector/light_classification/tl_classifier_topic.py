#!/usr/bin/env python
import rospy

from std_msgs.msg import Int32

from styx_msgs.msg import TrafficLight

"""This class subscribes to a topic to listen for the light status. This implementation can be used to 
   test the other parts of the code, without the specific traffic light classifier implementation.

   To make it work use the 'rostopic pub' command:
	rostopic pub /light_status std_msgs/Int32 2
   The value at the end (2 in this case) is NOT the enumeration value of the TrafficLight enumeration;
 	it's the predicted value as returned by the TensorFlow implementation (check the 
	tl_learning/label_map.pbtxt file for mapping)
"""
class TLClassifierTopic(object):
    def __init__(self):
	rospy.Subscriber('/light_status', Int32, self.light_status_cb)

	self.light_status = TrafficLight.UNKNOWN

    def light_status_cb(self, msg):
	if( msg.data == 1 ):
	    self.light_status = TrafficLight.GREEN
	elif( msg.data == 2 ):
	    self.light_status = TrafficLight.RED
	elif( msg.data == 3 ):
	    self.light_status = TrafficLight.YELLOW
	else:
	    self.light_status = TrafficLight.UNKNOWN

	rospy.loginfo('Status: %s; converted: %s', msg.data, self.light_status)

    def get_classification(self, image):
	return self.light_status

