import PyCapture2
import numpy as np
from constants import constants

"""
Utilities for pulling output from a PT-Grey camera.
"""


def initialize_camera(index):
    """
    Initializes a PT Grey camera by its enumerated index, and returns the camera instance
    :return: A camera instance representing the camera and its controls.
    """
    bus = PyCapture2.BusManager()
    camera = PyCapture2.Camera()
    uid = bus.getCameraFromIndex(index)
    camera.connect(uid)
    configure_camera(camera)
    camera.startCapture()
    return camera


def configure_camera(camera):
    """
    Configure the camera with our expected input
    :param camera:
    :return:
    """
    width, height = constants.IMAGE_WIDTH, constants.IMAGE_HEIGHT
    fmt7info, supported = camera.getFormat7Info(0)
    offset_x = int((fmt7info.maxWidth - width) / 2)
    offset_y = int((fmt7info.maxHeight - height) / 2)
    pxfmt = PyCapture2.PIXEL_FORMAT.RAW8
    fmt7imgSet = PyCapture2.Format7ImageSettings(0, offset_x, offset_y, width, height, pxfmt)
    fmt7pktInf, isValid = camera.validateFormat7Settings(fmt7imgSet)
    camera.setFormat7ConfigurationPacket(fmt7pktInf.recommendedBytesPerPacket, fmt7imgSet)

    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.AUTO_EXPOSURE, autoManualMode=True)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.SHARPNESS, autoManualMode=True)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.SHUTTER, autoManualMode=True)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.GAIN, autoManualMode=True)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.FRAME_RATE, autoManualMode=True)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.AUTO_EXPOSURE, onOff=True)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.FRAME_RATE, onOff=True, autoManualMode=False, absValue=40)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.GAMMA, onOff=True)
    camera.setProperty(type=PyCapture2.PROPERTY_TYPE.SHARPNESS, onOff=True)


def numpy_image_from_camera_image(pycap_image):
    """
    Converts a raw image from the camera to a usable format with OpenCV
    :param pycap_image:
    :return:
    """
    cv_image = np.asarray(pycap_image.getData()).reshape((pycap_image.getRows(), pycap_image.getCols())).astype(np.uint8)
    return cv_image
