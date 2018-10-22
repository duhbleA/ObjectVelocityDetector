#include <Python.h>
#include <iostream>
#include "/home/team0/.local/lib/python3.5/site-packages/numpy/core/include/numpy/ndarrayobject.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/conversion.h"

int main(int argc, char *argv[])
{
    // Initialize the Python Interpreter and add all of the local modules
    Py_Initialize();
    PyObject *sysPath = PySys_GetObject((char*)"path");
    PyList_Append(sysPath, PyUnicode_FromString("."));

    // Create the main module (main.py)
    PyObject* moduleString = PyUnicode_FromString((char*) "main");
    PyObject* mainModule = PyImport_Import(moduleString);


    if (mainModule != NULL)
    {
        // Create an object that can invoke the start() function defined in main.py
        PyObject* startFunction = PyObject_GetAttrString(mainModule ,(char*) "start");

        if (startFunction && PyCallable_Check(startFunction))
        {
            // Create a default blank argument and invoke start() defined in main.py
            PyObject* startArg = PyTuple_New(0);
            PyObject_CallObject(startFunction, startArg);
            Py_DECREF(startArg);
            Py_DECREF(startFunction);

            // Create an object that can invoke execute_session("1") defined in main.py
            PyObject* executeSession1 = PyObject_GetAttrString(mainModule ,(char*) "execute_session");
            PyObject* executeArg1 = PyTuple_Pack(1, PyUnicode_FromString((char*)"0"));

            // Create an object that can invoke execute_session("2") defined in main.py
            PyObject* executeSession2 = PyObject_GetAttrString(mainModule ,(char*) "execute_session");
            PyObject* executeArg2 = PyTuple_Pack(1, PyUnicode_FromString((char*)"1"));

            // Create an object that can invoke window_terminated_requested() defined in main.py
            PyObject* exitWindowTest = PyObject_GetAttrString(mainModule, (char*) "window_terminated_requested");
            PyObject* exitWindowArg = PyTuple_New(0);

            // The function result of window_terminated_requested
            PyObject* windowCheck;

            // Converts ndarray of numpy values to a cv::Mat
            NDArrayConverter cvt;
            cv::Mat matImage1;
            cv::Mat matImage2;
            while (true)
            {
                if (executeSession1 && PyCallable_Check(executeSession1)
                     && executeSession2 && PyCallable_Check(executeSession2))
                {

                    // Call execute_session, and convert its result to a cv::Mat
                    PyObject* ex1ret = PyObject_CallObject(executeSession1, executeArg1);
                    matImage1 = cvt.toMat(ex1ret);

                    // Display the image
                    cv::namedWindow( "Display window 1", cv::WINDOW_AUTOSIZE );
                    cv::imshow( "Display window 1", matImage1);


                    // Call execute_session, and convert its result to a cv::Mat
                    PyObject* ex2ret = PyObject_CallObject(executeSession2, executeArg2);
                    matImage2 = cvt.toMat(ex2ret);
                    // Display the image
                    cv::namedWindow( "Display window 2", cv::WINDOW_AUTOSIZE );
                    cv::imshow( "Display window 2", matImage2);

                    windowCheck = PyObject_CallObject(exitWindowTest, exitWindowArg);
                   
                    // Terminate the loop if q is pressed, but first clean and dereference
                    if (PyObject_IsTrue(windowCheck) || cv::waitKey(1) == 113)
                    {
                        Py_DECREF(exitWindowTest);
                        Py_DECREF(exitWindowArg);
                        Py_DECREF(executeSession1);
                        Py_DECREF(executeArg1);
                        Py_DECREF(executeSession2);
                        Py_DECREF(executeArg2);
                        Py_DECREF(windowCheck);
                        break;
                    }
                }
            }

            // Invoke the stop() method defined in main.py
            PyObject* stop = PyObject_GetAttrString(mainModule, (char*) "window_terminated_requested");
            PyObject* stopArg = PyTuple_New(0);
            PyObject_CallObject(stop, stopArg);

            Py_DECREF(stop);
            Py_DECREF(stopArg);

        }
    }
    // Terminate the python runtime
    Py_Finalize();
    return 0;
}
