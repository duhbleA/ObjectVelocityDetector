
import numpy as np
from constants import constants

"""
Helper methods to visualize output to a OpenCV instances to display the computer vision to the user
"""


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
