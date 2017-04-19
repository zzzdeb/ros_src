#ifndef _ANGLE_SEGMENT_2D_
#define _ANGLE_SEGMENT_2D_

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace anglesegment{

class AngleSegment2d{

    typedef pcl::PointXYZ           PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    typedef sensor_msgs::LaserScan LaserScan;

    public:
        AngleSegment2d(ros::NodeHandle* n, int angle_threshold);

        ~AngleSegment2d(){
            ROS_INFO("AngleSegment2d destructed");
        }


        bool angle_test(const LaserScan::Ptr& scan_msg, unsigned int i);
        void scan_callback(const LaserScan::ConstPtr& scan_msg);

    protected:
        int angle_threshold;

        ros::NodeHandle nh_;
        ros::Publisher scan_pub;
        ros::Subscriber scan_sub;

};

} // anglesegment namespace

#endif