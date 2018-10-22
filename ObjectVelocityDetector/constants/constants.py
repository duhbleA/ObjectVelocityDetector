# System wide constants

# Path information, names, and locations relating to the frozen inference graph being used for object detection
MODEL_NAME = "ssd_mobilenet_v1_coco_2018_01_28"
PATH_TO_FROZEN_GRAPH = MODEL_NAME + "/frozen_inference_graph.pb"
PATH_TO_OBJECT_LABELS = MODEL_NAME + "/mscoco_label_map.pbtxt"
MAX_NUMBER_OF_OBJECT_CLASSES = 90

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

DISPLAY_LIVE_OUTPUT = False
USE_BUILTIN_WEBCAM = True
