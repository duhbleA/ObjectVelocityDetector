
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

cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud<PointType> point_cloud, pcl::PointCloud<PointType> *visible_points)
{
    cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

    for (pcl::PointCloud<PointType>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
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

            //cv::circle(plane, xy, 3, intensity, -1);
            plane.at<float>(xy) = pt->intensity;
        }
    }

    cv::Mat plane_gray;
    cv::normalize(plane, plane_gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::dilate(plane_gray, plane_gray, cv::Mat());

    return plane_gray;
}

pcl::PointCloud<pcl::PointXYZ>* toPointsXYZ(pcl::PointCloud<PointType> point_cloud)
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

cv::Vec3b atf(cv::Mat rgb, cv::Point xy_f)
  {
    cv::Vec3i color_i;
    color_i.val[0] = color_i.val[1] = color_i.val[2] = 0;

    int x = xy_f.x;
    int y = xy_f.y;

    for (int row = 0; row <= 1; row++)
    {
      for (int col = 0; col <= 1; col++)
      {
        cv::Vec3b c = rgb.at<cv::Vec3b>(cv::Point(x + col, y + row));
        for (int i = 0; i < 3; i++)
        {
          color_i.val[i] += c.val[i];
        }
      }
    }

    cv::Vec3b color;
    for (int i = 0; i < 3; i++)
    {
      color.val[i] = color_i.val[i] / 4;
    }
    return color;
  }
  
  pcl::PointCloud<pcl::PointXYZRGB> colour(pcl::PointCloud<PointType> point_cloud, cv::Mat frame_rgb, cv::Mat P)
{
  pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
  for (pcl::PointCloud<PointType>::iterator pt = point_cloud.begin(); pt < point_cloud.end(); pt++)
  {
    cv::Point xy = project(*pt, P);

    cv::Vec3b rgb = atf(frame_rgb, xy);
    pcl::PointXYZRGB pt_rgb(rgb.val[2], rgb.val[1], rgb.val[0]);
    pt_rgb.x = pt->x;
    pt_rgb.y = pt->y;
    pt_rgb.z = pt->z;

    color_cloud.push_back(pt_rgb);
  }
  return color_cloud;
}
