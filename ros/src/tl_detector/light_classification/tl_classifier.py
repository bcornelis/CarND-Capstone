from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf

from PIL import Image

class TLClassifier(object):
    def __init__(self):

	"""Specify configuration"""
	PATH_TO_CKPT = '../../../tl_learning/frozen/frozen_inference_graph.pb'

	"""Load the trained model"""
	detection_graph = tf.Graph()
	with detection_graph.as_default():
	    od_graph_def = tf.GraphDef()
	    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
		serialized_graph = fid.read()
		od_graph_def.ParseFromString(serialized_graph)
		tf.import_graph_def(od_graph_def, name='')
	    self.sess = tf.Session(graph=detection_graph)

	self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
	self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
	self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
	self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')
	self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

	"""Warm up the system by testing on 2 images"""
	test_image = Image.open('./left0011.jpg')
	test_image_result = self.get_classification(test_image)
	assert test_image_result == 3 # Yellow
        test_image = Image.open('./left0568.jpg')
        test_image_result = self.get_classification(test_image)
        assert test_image_result == 1 # Green

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	result = TrafficLight.UNKNOWN

	"""Convert the image into the proper format"""
#	image_np = self.load_image_into_numpy_array(image)
	self.image_np_expanded = np.expand_dims(image, axis=0)

	"""Apply detection"""
	(boxes, scores, classes, num) = self.sess.run(
          [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
          feed_dict={self.image_tensor: self.image_np_expanded})

	"""And check if a green/Red light has been found"""
	if( num > 0):
	    #find the highest score in the scores list
	    max_score_idx = np.squeeze(scores).argmax()
	    # and get the class going with this score
	    result = np.squeeze(classes).astype(np.int32)[max_score_idx]

	return result

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)

