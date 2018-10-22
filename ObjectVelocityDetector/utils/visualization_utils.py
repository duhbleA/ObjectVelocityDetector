from object_detection.utils import visualization_utils as vis_util
import numpy as np
from constants import constants

"""
Helper methods to visualize output to a OpenCV instances to display the computer vision to the user
"""


def visualize_output(cv2, image_to_show, boxes, classes, scores, category_indices, camera_id):
    """
    Visualizes output to the user through a OpenCV window
    :param cv2: Instance of OpenCV
    :param image_to_show: The numpy image to display
    :param boxes: The location of the boxes of the object detection
    :param classes: The different possible classes of objects shown in the frame of detection
    :param scores: The likelihood score for each detected object
    :param category_indices: The index of the common labels for possible objects the graph can see.
    :return:
    """
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_to_show,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        category_indices,
        use_normalized_coordinates=True,
        line_thickness=8)

    # Display output for fun
    cv2.imshow("Camera #" + camera_id, cv2.resize(image_to_show, (constants.IMAGE_WIDTH, constants.IMAGE_HEIGHT)))


def wait_for_q(cv2):
    """
    Waits for input and returns true when q is pressed in a open Open CV image.

    :param cv2: Instance of OpenCV
    :return: True when q is pressed
    """
    # Kill the window by pressing q
    if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        return True
