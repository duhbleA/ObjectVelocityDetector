#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__

#include <math.h>

constexpr float Rad2Deg = 180.0 / M_PI;

inline float GetXYCameraAngle(const pcl::PointXYZI &point)
{
    return std::atan2(point.y, point.x) * Rad2Deg;
}

inline float GetXZCameraAngle(const pcl::PointXYZI &point)
{
    return std::atan2(point.z, point.x) * Rad2Deg;
}

inline float GetXYZDistance(const pcl::PointXYZI &point)
{
    return std::sqrt((point.x * point.x) +
                     (point.y * point.y) +
                     (point.z * point.z));
}

inline float GetXYDistance(const pcl::PointXYZI &point)
{
    return std::sqrt(point.x * point.x +
                     point.y * point.y);
}

inline float GetXZDIstance(const pcl::PointXYZI &point)
{
    return std::sqrt(point.x * point.x +
                     point.z * point.z);
}

inline float GetXYZDistance(const pcl::PointXYZI &pointA, const pcl::PointXYZI &pointB)
{
    return std::sqrt(std::pow((pointB.x - pointA.x), 2) +
                     std::pow((pointB.y - pointA.y), 2) +
                     std::pow((pointB.z - pointA.z), 2));
//    return std::sqrt((pointB.x - pointA.x) * (pointB.x - pointA.x) +
//                     (pointB.y - pointA.y) * (pointB.y - pointA.y) +
//                     (pointB.z - pointA.z) * (pointB.z - pointA.z));
}


#endif
