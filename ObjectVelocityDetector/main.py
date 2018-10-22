
import tensorflow as tf
from utils import graph_utils, visualization_utils
from constants import constants
if constants.USE_BUILTIN_WEBCAM == False:
    from utils import camera_utils
import cv2
import time
import numpy as np
import cython_utils

"""
Main Program for CPE 656 Team 0.

Fuses lidar and image data to generate velocity vectors of detected objects.
"""

camera_1 = None
camera_2 = None
session = None
category_index = None


def start():
    """
    Loads the tensor flow frozen graph, the categorization indices, the TensorFlow sessions, and the PTGrey cameras.

    :return: None
    """
    obj_detection_graph = graph_utils.load_model(tf, constants.PATH_TO_FROZEN_GRAPH)

    global category_index
    category_index = graph_utils.get_category_indices(constants.PATH_TO_OBJECT_LABELS,
                                                      constants.MAX_NUMBER_OF_OBJECT_CLASSES)

    # create sessions for each graph
    global session
    session = tf.Session(graph=obj_detection_graph)

    global camera_1
    global camera_2
    if constants.USE_BUILTIN_WEBCAM == False:

        # Initialize the PyCapture capture
        camera_1 = camera_utils.initialize_camera(0)
        camera_2 = camera_utils.initialize_camera(1)
    else:
        camera_1 = cv2.VideoCapture(0)


def stop():
    """
    Stops cameras.

    :return: None
    """
    if constants.USE_BUILTIN_WEBCAM == False:
        camera_1.stopCapture()
        camera_2.stopCapture()


def execute_session(camera_id):
    """
    Runs a session of Tensor flow on a particular image frame.

    :param camera_id The camera id to pull the capture from.
    """
    time1 = time.time()

    # Create a numpy image

    image = None
    if constants.USE_BUILTIN_WEBCAM == False:
        if camera_id == "1":
            image = camera_1.retrieveBuffer()
        elif camera_id == "0":
            image = camera_2.retrieveBuffer()
    else:
        ret, image = camera_1.read()

    image_np = None
    if image is not None:
        if constants.USE_BUILTIN_WEBCAM == False:
            image_np = cython_utils.tonumpyarray(image.getData(), constants.IMAGE_HEIGHT, constants.IMAGE_WIDTH)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BAYER_BG2BGR)
        else:
            image_np = image

        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = session.graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = session.graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = session.graph.get_tensor_by_name('detection_scores:0')
        classes = session.graph.get_tensor_by_name('detection_classes:0')

        num_detections = session.graph.get_tensor_by_name('num_detections:0')

        # Actual detection from the result of the neural network
        (boxes, scores, classes, num_detections) = session.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        # Display live output if the constants boolean was set
        if constants.DISPLAY_LIVE_OUTPUT:
            # Visualize output in CV 2 window
            visualization_utils.visualize_output(cv2, image_np, boxes, classes, scores, category_index, camera_id)

        time2 = time.time()
        print("Camera #" + camera_id + " process time: " + str(time2 - time1))

        return image_np


def window_terminated_requested():
    """
    Terminates an OpenCV window by pressing q.  Used because OpenCV has serious issues without a termination
    loop check.

    :return: True if user requested to exit OpenCV window.
    """
    if constants.DISPLAY_LIVE_OUTPUT:
        # Wait for q to close the window
        return visualization_utils.wait_for_q(cv2)

