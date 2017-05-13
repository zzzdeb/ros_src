#ifndef _ANGLE_SEGMENTER_
#define _ANGLE_SEGMENTER_

#include <ros/ros.h>
#include <ros/console.h>

#include <queue>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image_spherical.h>

#include <opencv/cv.h>

namespace anglesegment
{

class AngleSegmenter
{

    typedef sensor_msgs::PointCloud2 PointCloudT;

  public:
    AngleSegmenter(const pcl::RangeImageSpherical& rcloud, int angle_threshold);

    ~AngleSegmenter()
    {
        ROS_INFO("AngleSegmenter destructed");
    }
    void segment();
    bool angle_test(const LaserScan::Ptr &scan_msg, unsigned int i);

  protected:
    int angle_threshold;
    pcl::RangeImageSpherical::Ptr rcloud;
    cv::Mat _label_image;
};

} // anglesegment namespace

#endif