#ifndef __PCLUTILS_H__
#define __PCLUTILS_H__

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <opencv2/core/core.hpp>

#include <vector>
#include <chrono>
#include "../include/mathutils.h"

typedef pcl::PointXYZI PointType;


struct Sample
{
    cv::Rect boundingBox;
    float objectDistance;
    int pointsInside;
    int64_t sampleTime;
    double velocity;
    bool isVisible;
    int framesSinceLastSeen;

};


void GetMinMaxPoints(const std::vector<PointType> &visible_points, float &min, float &max)
{
    max = 0;
    min = 1000000.0;
    float distance = 0;
    for(const PointType &point : visible_points)
    {
        distance = GetXYZDistance(point);
        if(distance > max)
            max = distance;
        if(distance < min)
            min = distance;
    }
}

cv::Point2f project(const PointType &pt, const cv::Mat &projection_matrix)
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
    return cv::Point2f(x, y);
}

cv::Vec3b generateHeatMap(float minDistance, float interval, const PointType& point)
{
    float distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);

    float minRange = minDistance;
    float maxRange = minDistance + interval;

    if ((distance >= minRange) && (distance <= maxRange))
    {
        return cv::Vec3b(0, 0, 255);
    }

    minRange = maxRange;
    maxRange = maxRange + interval;
    if ((distance > minRange) && (distance <= maxRange))
    {
        return cv::Vec3b(0, 128, 255);
    }

    minRange = maxRange;
    maxRange = maxRange + interval;
    if ((distance > minRange) && (distance <= maxRange))
    {
        return cv::Vec3b(0, 255, 255);
    }

    minRange = maxRange;
    maxRange = maxRange + interval;
    if ((distance > minRange) && (distance <= maxRange))
    {
        return cv::Vec3b(0, 255, 0);
    }

    minRange = maxRange;
    maxRange = maxRange + interval;
    if ((distance > minRange) && (distance <= maxRange))
    {
        return cv::Vec3b(255, 0, 0);
    }

    minRange = maxRange;
    maxRange = maxRange + interval;
    if ((distance > minRange) && (distance <= maxRange))
    {
        return cv::Vec3b(255, 102, 102);
    }

    minRange = maxRange;
    maxRange = maxRange + interval;
    if ((distance > minRange) && (distance <= maxRange))
    {
        return cv::Vec3b(255, 204, 204);
    }

    return cv::Vec3b(255, 255, 255);
}

void project(cv::Mat& projection_matrix, cv::Rect& frame, cv::Mat& image, const std::vector<PointType> &visible_points, float min, float max)
{
    //cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

    if (visible_points.size() != 0)
    {

        float interval = (max - min) / 7.0f;

        for (const PointType &pt : visible_points)
        {
            cv::Point2f xy = project(pt, projection_matrix);


            image.at<cv::Vec3b>(xy) = generateHeatMap(min, interval, pt);
        }
    }
}

void FilterBoundingBox(const std::vector<PointType> &visiblePoints, const cv::Mat& projection_matrix, cv::Mat boundingBoxes, std::vector<PointType> &BoundPoints, float DistanceThreshold,
                       std::vector<Sample> &samples, std::vector<Sample> &samplesToRender)
{
    float x1, x2, y1, y2;

    if(boundingBoxes.rows == 0)
    {
        return;
    }
    if(boundingBoxes.cols == 0)
    {
        return;
    }

    std::vector<Sample> workingSamples;
    for(int i = 0; i < boundingBoxes.rows; i ++)
    {
        const auto &box = boundingBoxes.row(i);

        if(box.cols == 4)
        {
            if(!box.col(0).empty() && !box.col(1).empty() && !box.col(2).empty() && !box.col(3).empty())
            {
                y1 = box.col(0).at<float>();
                x1 = box.col(1).at<float>();
                y2 = box.col(2).at<float>();
                x2 = box.col(3).at<float>();

                cv::Rect_<float> bound_box_rect = cv::Rect_<float>(cv::Point2f(x2, y2), cv::Point2f(x1, y1));

                Sample boundingBoxSample;
                boundingBoxSample.boundingBox = bound_box_rect;
                boundingBoxSample.sampleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                boundingBoxSample.objectDistance = 0.0f;
                boundingBoxSample.pointsInside = 0;
                boundingBoxSample.velocity = 0.0;
                boundingBoxSample.isVisible = false;
                boundingBoxSample.framesSinceLastSeen = 0;

                for (const PointType &point : visiblePoints)
                {
                    cv::Point2f xy = project(point, projection_matrix);
                    if(xy.inside(bound_box_rect))
                    {
                        float distance = GetXYZDistance(point);
                        if(DistanceThreshold > 0.0)
                        {
                            if(distance < DistanceThreshold)
                            {
                                BoundPoints.push_back(point);
                                boundingBoxSample.pointsInside += 1;
                            }
                        }
                        else
                        {
                            BoundPoints.push_back(point);
                            boundingBoxSample.pointsInside += 1;
                        }

                        boundingBoxSample.objectDistance += distance;
                    }
                }
                workingSamples.push_back(boundingBoxSample);
            }
        }

    }

    std::vector<Sample> visibleSamples;
    for (Sample &newSample : workingSamples)
    {
        newSample.objectDistance = newSample.objectDistance / (float) newSample.pointsInside;

        int indexToReplace = -1;
        for (int i = 0; i < samples.size(); i++)
        {
            Sample oldSample = samples[i];

            float areaRatio = (float) newSample.boundingBox.area() / (float) oldSample.boundingBox.area();

            if (areaRatio >= 0.80f && areaRatio <= 1.2f)
            {
                if ((newSample.boundingBox & oldSample.boundingBox).area() > 0)
                {
                    newSample.velocity = (double) (newSample.objectDistance - oldSample.objectDistance) / (double) ((newSample.sampleTime - oldSample.sampleTime) / 1000.0);
                    indexToReplace = i;
                    newSample.isVisible = true;
                    break;
                }
            }
        }
        if (indexToReplace != -1)
        {
            samples[indexToReplace] = newSample;
            visibleSamples.push_back(newSample);
        }
        else
        {
            newSample.isVisible = true;
            visibleSamples.push_back(newSample);
            samples.push_back(newSample);
        }
    }

    for (Sample &sample : visibleSamples)
    {
        samplesToRender.push_back(sample);
    }
}

void TrimPoints(const pcl::PointCloud<PointType>::ConstPtr cloud, cv::Rect &frame, const cv::Mat& projection_matrix, std::vector<PointType> &visible_points,
                float &min, float &max)
{
    max = 0.0f;
    min = 1000000000.0f;
    float distance = 0.0;

    for (const PointType &pt : *cloud)
    {
        bool rendered = false;
        // behind the camera
        if (pt.z < 0)
        {
            continue;
        }

        //float intensity = pt->intensity;

        cv::Point2f xy = project(pt, projection_matrix);
        if (xy.inside(frame))
        {
            visible_points.push_back(pt);

            distance = GetXYZDistance(pt);
            if(max < distance)
                max = distance;
            if(min > distance)
                min = distance;
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


