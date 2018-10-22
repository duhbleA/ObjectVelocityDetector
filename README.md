# ObjectVelocityDetector

Performs sensor fusion using a VLP-16 lidar and a BlackFly camera to determine the velocities of detected objects from TensorFlow.

System requirements:
-Ubuntu 16.04
-Python 3.x
-TensorFlow
-One of the object detection models from Tensor Flow's model zoo

Required linux library packages:
-opencv
-opencv-dev
-libpython3-dev
-libpython
-python3-dev
-pcl 1.8x (will need to be compiled for 16.04)

Required Python3.x packages (using pip3):
-python-opencv
-tensorflow
-numpy
-Cython
-contextlib2
-pillow
-lxml
-jupyter
-matplotlib

Numpy's header is often hard to find.  Be sure to modify the include/conversions.h and src/main.cpp include statements to the proper path of ndarrayobject.h

InstallPCL.sh will download all required dependencies, compile and install the PCL library.

The expand_usb_core_mem.sh script must be used to expand the USB buffer memory before running this program, or else segmentation faults will occur.
