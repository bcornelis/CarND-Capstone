from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	result = TrafficLight.UNKNOWN

	image_red = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	image_red_range1 = cv2.inRange(image_red, np.array([0,50,50]) , np.array([10,255,255]))
	image_red_range2 = cv2.inRange(image_red, np.array([170,50,50]) , np.array([180,255,255]))

	converted_img = cv2.addWeighted(image_red_range1, 1.0, image_red_range2, 1.0, 0.0)
	blur_img = cv2.GaussianBlur(converted_img,(15,15),0)
	circles = cv2.HoughCircles(blur_img,cv2.HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=150)

	if circles is not None:
            result = TrafficLight.RED
        
	return result
