#ifndef _ANGLE_SEGMENTER_
#define _ANGLE_SEGMENTER_

#include <ros/ros.h>
#include <ros/console.h>

#include <queue>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>

namespace anglesegment
{

class AngleSegmenter
{

    typedef sensor_msgs::PointCloud2 PointT;

  public:
    AngleSegmenter(pclcloud, int angle_threshold);

    ~AngleSegmenter()
    {
        ROS_INFO("AngleSegmenter destructed");
    }
    void segment();
    bool angle_test(const LaserScan::Ptr &scan_msg, unsigned int i);

  protected:
    int angle_threshold;
    pcl::RangeImage::Ptr rcloud;
    vector<pclcloudptr> clouds;
};

} // anglesegment namespace

#endif