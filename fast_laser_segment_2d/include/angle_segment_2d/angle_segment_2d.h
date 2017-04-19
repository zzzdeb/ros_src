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
        AngleSegment2d(ros::NodeHandle* n);

        ~AngleSegment2d(){
            ROS_INFO("AngleSegment2d destructed");
        }

        void scan_callback(const LaserScan::ConstPtr& scan_msg);

    protected:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        ros::NodeHandle nh_;
        ros::Publisher scan_pub;
        ros::Subscriber scan_sub;

};

} // anglesegment namespace