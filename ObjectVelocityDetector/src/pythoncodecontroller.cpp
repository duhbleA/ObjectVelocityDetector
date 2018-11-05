#include "../include/pythoncodecontroller.h"

PythonCodeController::PythonCodeController()
{
    // Append the local modules to the python path
    PyObject *sysPath = PySys_GetObject((char *) "path");
    PyList_Append(sysPath, PyUnicode_FromString((char *) "."));


    // Create the main module (main.py)
    PyObject *moduleString = PyUnicode_FromString((char *) "main");
    PyObject *mainModule = PyImport_Import(moduleString);

    if (mainModule != nullptr)
    {
        // Create the object that will invoke start() in main.py
        startFunction = PyObject_GetAttrString(mainModule, (char *) "start");
        startArg = PyTuple_New(0);

        // Create an object that can invoke execute_session("1") defined in main.py
        executeSession1Function = PyObject_GetAttrString(mainModule, (char *) "execute_session");
        executeArg1 = PyTuple_Pack(1, PyUnicode_FromString((char *) "0"));

        // Create an object that can invoke execute_session("2") defined in main.py
        executeSession2Function = PyObject_GetAttrString(mainModule, (char *) "execute_session");
        executeArg2 = PyTuple_Pack(1, PyUnicode_FromString((char *) "1"));

        // Create an object that can invoke get_image() from main.py
        getImageFunction = PyObject_GetAttrString(mainModule, (char *) "get_image");
        getImageArg = PyTuple_New(0);

        // Create an object that can invoke get_boxes() from main.py
        getBoxesFunction = PyObject_GetAttrString(mainModule, (char *) "get_boxes");
        getBoxesArg = PyTuple_New(0);
        
        // Create an object that can invoke stop() from main.py
        stopFunction = PyObject_GetAttrString(mainModule, (char *) "stop");
        stopArg = PyTuple_New(0);
    }
    else
    {
        std::cout << "Main is ded" << std::endl;
        throw("main.py could not be found. Be sure to run the program from the same directory as main.py");
    }
}

PythonCodeController::~PythonCodeController()
{
    Py_XDECREF(executeSession1Function);
    Py_XDECREF(executeArg1);
    Py_XDECREF(executeSession2Function);
    Py_XDECREF(executeArg2);
    Py_XDECREF(getBoxesFunction);
    Py_XDECREF(getBoxesArg);
    Py_XDECREF(getImageFunction);
    Py_XDECREF(getBoxesFunction);
    Py_XDECREF(startFunction);
    Py_XDECREF(startArg);
    Py_XDECREF(stopFunction);
    Py_XDECREF(stopArg);
    Py_XDECREF(raw_image);
    Py_XDECREF(bounding_boxes);
}

void PythonCodeController::start()
{
    PyObject_CallObject(startFunction, startArg);
}

void PythonCodeController::terminate()
{
    PyObject_CallObject(stopFunction, stopArg);
}

cv::Mat PythonCodeController::imageFromLastSpin()
{
    return cvt.toMat(raw_image);
}

cv::Mat PythonCodeController::boxesFromLastSpin()
{
    return cvt.toMat(bounding_boxes);
}

void PythonCodeController::spinOnCamera1()
{
    PyObject_CallObject(executeSession1Function, executeArg1);
    raw_image = PyObject_CallObject(getImageFunction, getImageArg);
    bounding_boxes = PyObject_CallObject(getBoxesFunction, getBoxesArg);
}

void PythonCodeController::spinOnCamera2()
{
    PyObject_CallObject(executeSession2Function, executeArg2);
    raw_image = PyObject_CallObject(getImageFunction, getImageArg);
    bounding_boxes = PyObject_CallObject(getBoxesFunction, getBoxesArg);
}

