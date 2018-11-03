#ifndef __PCLUTILS_H__
#define __PCLUTILS_H__

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <opencv2/core/core.hpp>

#include "../include/mathutils.h"

typedef pcl::PointXYZI PointType;

cv::Point project(const PointType &pt, const cv::Mat &projection_matrix)
{
    //cv::Point2f xy = projectf(pt, projection_matrix);
    cv::Mat pt_3D(4, 1, CV_32FC1);

    pt_3D.at<float>(0) = pt.x;
    pt_3D.at<float>(1) = pt.y;
    pt_3D.at<float>(2) = pt.z;
    pt_3D.at<float>(3) = 1.0f;

    cv::Mat pt_2D = projection_matrix * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;
    return cv::Point(x, y);
}

cv::Vec3b generateHeatMap(float minDistance, float interval, const PointType& point)
{
    float distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);

    if ((distance >= minDistance) && (distance <= (minDistance + interval)))
    {
        return cv::Vec3b(0, 0, 255);
    }
    else if ((distance > (minDistance + interval)) && (distance <= (minDistance + 2*interval)))
    {
        return cv::Vec3b(0, 128, 255);
    }
    else if ((distance > (minDistance + 2*interval)) && (distance <= (minDistance + 3*interval)))
    {
        return cv::Vec3b(0, 255, 255);
    }
    else if ((distance > (minDistance + 3*interval)) && (distance <= (minDistance + 4*interval)))
    {
        return cv::Vec3b(0, 255, 0);
    }
    else if ((distance > (minDistance + 4*interval)) && (distance <= (minDistance + 5*interval)))
    {
        return cv::Vec3b(255, 0, 0);
    }
    else if ((distance > (minDistance + 5*interval)) && (distance <= (minDistance + 6*interval)))
    {
        return cv::Vec3b(255, 102, 102);
    }
    else if ((distance > (minDistance + 6*interval)) && (distance <= (minDistance + 7*interval)))
    {
        return cv::Vec3b(255, 204, 204);
    }
    else
    {
        return cv::Vec3b(255, 255, 255);
    }
    
    return cv::Vec3b(255, 255, 255);
}

void project(cv::Mat& projection_matrix, cv::Rect& frame, const pcl::PointCloud<PointType>* point_cloud, cv::Mat& image, pcl::PointCloud<PointType> *visible_points)
{
    //cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);


    for (pcl::PointCloud<PointType>::const_iterator pt = point_cloud->points.begin(); pt < point_cloud->points.end(); pt++)
    {

        // behind the camera
        if (pt->z < 0)
        {
            continue;
        }

        //float intensity = pt->intensity;
        cv::Point xy = project(*pt, projection_matrix);
        if (xy.inside(frame))
        {
            if (visible_points != NULL)
            {
                visible_points->push_back(*pt);
            }
        }
    }

    if (visible_points != NULL)
    {
        PointType minPt;
        PointType maxPt;
        pcl::getMinMax3D(*visible_points, minPt, maxPt);

        float alpha = GetXYZDistance(minPt);
        float beta = GetXYZDistance(maxPt);
        
        float minDistance = (alpha < beta) ? alpha : beta;
        float maxDistance = (alpha > beta) ? alpha : beta;
        
        float interval = (maxDistance - minDistance) / 7.0f;

        for (pcl::PointCloud<PointType>::iterator pt = visible_points->points.begin(); pt < visible_points->points.end(); pt++)
        {
            cv::Point xy = project(*pt, projection_matrix);
            
            
            image.at<cv::Vec3b>(xy) = generateHeatMap(minDistance, interval, *pt);
        }
    }
}

pcl::PointCloud<pcl::PointXYZ>* toPointsXYZ(pcl::PointCloud<PointType>& point_cloud)
{
    pcl::PointCloud<pcl::PointXYZ> *new_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    for (pcl::PointCloud<PointType>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
    {
        new_cloud->push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
    }
    return new_cloud;
}

void performTransform(const pcl::PointCloud<PointType>& point_cloud, pcl::PointCloud<PointType>& point_cloud_out, float x, float y, float z, float rottyx, float rottyy, float rottyz)
{
    Eigen::Affine3f transf = pcl::getTransformation(x, y, z, rottyx, rottyy, rottyz);
    pcl::PointCloud<PointType> new_cloud;
    pcl::transformPointCloud(point_cloud, point_cloud_out, transf);
}

#endif


