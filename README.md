# ObjectVelocityDetector

Performs sensor fusion using a VLP-16 lidar and a BlackFly camera to determine the velocities of detected objects from TensorFlow.

System requirements:
-Ubuntu 16.04
-Python 2.7
-TensorFlow
-One of the object detection models from Tensor Flow's model zoo

Required linux library packages:
*opencv
*opencv-dev
*libpython-dev
*libpython
*python-dev
*pcl 1.8x (will need to be compiled for 16.04)
*python-matplotlib

Required Python2.7 packages (using pip3):
*python-opencv
*tensorflow
*numpy
*Cython
*contextlib2
*pillow
*lxml

InstallPCL.sh will download all required dependencies, compile and install the PCL library.

The expand_usb_core_mem.sh script must be used to expand the USB buffer memory before running this program, or else segmentation faults will occur.
