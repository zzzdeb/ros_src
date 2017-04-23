#ifndef _ANGLE_SEGMENT_3D_VIZ_
#define _ANGLE_SEGMENT_3D_VIZ_

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace anglesegment
{

class AngleSegment3dViz
{

    typedef sensor_msgs::LaserScan LaserScan;

  public:
    AngleSegment3dViz(ros::NodeHandle *n, int angle_threshold);

    ~AngleSegment3dViz()
    {
        ROS_INFO("AngleSegment3dViz destructed");
    }

    bool angle_test(const LaserScan::Ptr &scan_msg, unsigned int i);
    void scan_callback(const LaserScan::ConstPtr &scan_msg);

  protected:
    int angle_threshold;

    ros::NodeHandle nh_;
    ros::Publisher scan_pub;
    ros::Subscriber scan_sub;

    sensor_msgs::LaserScan::ConstPtr prev_scan_msg_;
    LaserScan::Ptr leer_scan;
};

} // anglesegment namespace

#endif