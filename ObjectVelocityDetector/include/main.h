#ifndef OBJECTVELOCITYDETECTOR_MAIN_H
#define OBJECTVELOCITYDETECTOR_MAIN_H


#include "Python.h"
#include <iostream>
#include "numpy/ndarrayobject.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/conversion.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

using namespace cv;
using namespace Eigen;
using namespace std;

typedef pcl::PointXYZI PointType;


float left_raw_projection[12] = {942.129700f, 0.0, 985.634114f, 0.0,
                                 0.0, 1060.674438f, 600.441036f, 0.0,
                                 0.0, 0.0, 1.0f, 0.0};
float right_raw_projection[12] = {991.082031f, 0.000000, 965.097550f, 0.000000,
                                  0.000000, 1151.225708f, 630.262194f, 0.000000,
                                  0.000000, 0.000000, 1.000000, 0.000000};

Mat *left_projection_matrix = new Mat(3, 4, CV_32FC1, &left_raw_projection);
Mat *right_projection_matrix = new Mat(3, 4, CV_32FC1, &right_raw_projection);

Matrix4f left_rigid_body_transformation;
Matrix4f right_rigid_body_transformation;


#endif
