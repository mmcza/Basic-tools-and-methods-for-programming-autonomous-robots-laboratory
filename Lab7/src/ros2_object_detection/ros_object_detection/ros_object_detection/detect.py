#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import six
import tensorflow as tf
import numpy as np
import os
import time
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from interface_tutorial.msg import Num
from ros_object_detection_msgs.msg import BoundingBoxes, BoundingBox
import threading

# patch tf1 into `utils.ops`
utils_ops.tf = tf.compat.v1

# Patch the location of gfile
tf.gfile = tf.io.gfile


def version(v):
    return tuple(map(int, (v.split("."))))


if version(tf.__version__) < version('2.0.0'):
    tf.enable_eager_execution()


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.declare_parameter('model')
        self.declare_parameter('label_map')
        self.declare_parameter('score_thresh')
        model_p = self.get_parameter('model').value
        label_map_p = self.get_parameter('label_map').value

        self.model = self.load_model(model_p)
        self.category_index = label_map_util.create_category_index_from_labelmap(label_map_p, use_display_name=True)
        self.min_score = self.get_parameter('score_thresh').value

        self.image_pub = self.create_publisher(Image, '/image_annotated', 1)
        self.box_pub = self.create_publisher(BoundingBoxes, '/bounding_box', 1)
        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        #     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        #     depth=1
        # )
        self.sub = self.create_subscription(Image, '/traffic/image', self.get_image,
                                            qos_profile=qos_profile_sensor_data)
        self.subscribe_lock = False
        self.image_msg = None

    def load_model(self, model_dir):
        model_dir = os.path.join(model_dir, 'saved_model')
        model = tf.compat.v2.saved_model.load(model_dir, None)
        # model = tf.saved_model.load(model_dir)
        model = model.signatures['serving_default']

        return model

    def run_inference_for_single_image(self, image):
        image = np.asarray(image)
        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]

        # Run inference
        output_dict = self.model(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(output_dict.pop('num_detections'))
        output_dict = {key: value[0, :num_detections].numpy()
                       for key, value in output_dict.items()}
        output_dict['num_detections'] = num_detections

        # detection_classes should be ints.
        output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)

        # Handle models with masks:
        if 'detection_masks' in output_dict:
            # Reframe the the bbox mask to the image size.
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                output_dict['detection_masks'], output_dict['detection_boxes'],
                image.shape[0], image.shape[1])
            detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                               tf.uint8)
            output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()

        return output_dict

    def annotate_inference(self, image):
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        # Actual detection.
        output_dict = self.run_inference_for_single_image(image)
        # Visualization of the results of a detection.
        output_image = np.copy(image)
        vis_util.visualize_boxes_and_labels_on_image_array(
            output_image,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            self.category_index,
            instance_masks=output_dict.get('detection_masks_reframed', None),
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=self.min_score)

        boxes, classes, scores = list(), list(), list()

        for box, cls, score in zip(output_dict['detection_boxes'], output_dict['detection_classes'],
                                   output_dict['detection_scores']):
            if score >= self.min_score:
                if cls in six.viewkeys(self.category_index):
                    class_name = self.category_index[cls]['name']
                    boxes.append(box)
                    classes.append(class_name)
                    scores.append(score)

        return output_image, boxes, classes, scores

    def get_image(self, data):
        if self.subscribe_lock is False:
            self.subscribe_lock = True
            self.image_msg = data
            self.subscribe_lock = False

    def detect(self):
        if self.image_msg is not None:
            if self.subscribe_lock is False:
                self.subscribe_lock = True
                bridge = CvBridge()
                image = bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='passthrough')
                annotated, boxes, classes, scores = self.annotate_inference(image)
                message = bridge.cv2_to_imgmsg(annotated)
                self.image_pub.publish(message)

                box_data = list()

                for box, cls, score in zip(boxes, classes, scores):
                    box_msg = BoundingBox()
                    box_msg.data = cls
                    box_msg.confidence = float(score)
                    box_msg.xmin = float(box[1])
                    box_msg.ymin = float(box[0])
                    box_msg.xmax = float(box[3])
                    box_msg.ymax = float(box[2])
                    box_data.append(box_msg)
                boxes_msg = BoundingBoxes()
                boxes_msg.data = box_data
                self.box_pub.publish(boxes_msg)

                self.image_msg = None
                self.subscribe_lock = False
        else:
            print('Not Entered')


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetectionNode()
    rate = node.create_rate(100)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    while rclpy.ok():
        node.detect()
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
