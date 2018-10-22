#include <Python.h>
#include <iostream>
#include "/home/team0/.local/lib/python3.5/site-packages/numpy/core/include/numpy/ndarrayobject.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/conversion.h"

int main(int argc, char *argv[])
{
    // Initialize the Python Interpreter
    Py_Initialize();
    PyObject *sysPath = PySys_GetObject((char*)"path");
    PyList_Append(sysPath, PyUnicode_FromString("."));


    PyObject* moduleString = PyUnicode_FromString((char*) "main");
    PyObject* mainModule = PyImport_Import(moduleString);


    if (mainModule != NULL)
    {
        PyObject* startFunction = PyObject_GetAttrString(mainModule ,(char*) "start");

        if (startFunction && PyCallable_Check(startFunction))
        {
            PyObject* startArg = PyTuple_New(0);
            PyObject_CallObject(startFunction, startArg);
            Py_DECREF(startArg);
            Py_DECREF(startFunction);

            PyObject* executeSession1 = PyObject_GetAttrString(mainModule ,(char*) "execute_session");
            PyObject* executeArg1 = PyTuple_Pack(1, PyUnicode_FromString((char*)"0"));

            PyObject* executeSession2 = PyObject_GetAttrString(mainModule ,(char*) "execute_session");
            PyObject* executeArg2 = PyTuple_Pack(1, PyUnicode_FromString((char*)"1"));

            PyObject* exitWindowTest = PyObject_GetAttrString(mainModule, (char*) "window_terminated_requested");
            PyObject* exitWindowArg = PyTuple_New(0);

            PyObject* windowCheck;

            NDArrayConverter cvt;
            cv::Mat matImage1;
            cv::Mat matImage2;
            while (true)
            {
                if (executeSession1 && PyCallable_Check(executeSession1)
                     && executeSession2 && PyCallable_Check(executeSession2))
                {

                    PyObject* ex1ret = PyObject_CallObject(executeSession1, executeArg1);
                    matImage1 = cvt.toMat(ex1ret);
                    cv::namedWindow( "Display window 1", cv::WINDOW_AUTOSIZE );
                    cv::imshow( "Display window 1", matImage1);


                    PyObject* ex2ret = PyObject_CallObject(executeSession2, executeArg2);
                    matImage2 = cvt.toMat(ex2ret);
                    cv::namedWindow( "Display window 2", cv::WINDOW_AUTOSIZE );
                    cv::imshow( "Display window 2", matImage2);

                    windowCheck = PyObject_CallObject(exitWindowTest, exitWindowArg);

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

            PyObject* stop = PyObject_GetAttrString(mainModule, (char*) "window_terminated_requested");
            PyObject* stopArg = PyTuple_New(0);
            PyObject_CallObject(stop, stopArg);

            Py_DECREF(stop);
            Py_DECREF(stopArg);

        }
    }
    Py_Finalize();
    return 0;
}
