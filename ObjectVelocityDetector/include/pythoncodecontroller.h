#ifndef PYTHONCODECONTROLLER_H
#define PYTHONCODECONTROLLER_H

#include "Python.h"
#include "numpy/ndarrayobject.h"
#include "../include/conversion.h"

class PythonCodeController
{
public:

    PythonCodeController();

    ~PythonCodeController();

    // Spins the execute_session calls in python which pull images from the camers, and passes their images
    // through object detection.
    void spinOnCamera1();
    void spinOnCamera2();

    // Retrieval methods after a spin
    cv::Mat imageFromLastSpin();
    cv::Mat boxesFromLastSpin();

    // Start and stop the camera drivers in Python
    void start();
    void terminate();



private:
    // Object representations of python functions in main.py
    PyObject* executeSession1Function;
    PyObject* executeSession2Function;
    PyObject* getImageFunction;
    PyObject* getBoxesFunction;
    PyObject* stopFunction;
    PyObject* startFunction;

    // Parameter representations for python functions in main.py
    PyObject* executeArg1;
    PyObject* executeArg2;
    PyObject* getImageArg;
    PyObject* getBoxesArg;
    PyObject* stopArg;
    PyObject* startArg;

    // Raw_image and bounding_boxes from a spin
    PyObject* raw_image;
    PyObject* bounding_boxes;

    // Easy converter to cv:: Mat objects
    NDArrayConverter cvt;
};

#endif
