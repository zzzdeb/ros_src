#ifndef _ANGLE_SEGMENT_2D_PUB_
#define _ANGLE_SEGMENT_2D_PUB_

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
using namespace std;
namespace anglesegment
{

class AngleSegment2dPub
{

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    // typedef boost::shared_ptr<vector<vector<int> > >
    //typedef boost::shared_ptr<vector<vector<int> >
    typedef sensor_msgs::LaserScan LaserScan;

  public:
    AngleSegment2dPub(ros::NodeHandle *n, int angle_threshold);

    ~AngleSegment2dPub()
    {
        ROS_INFO("AngleSegment2dPub destructed");
    }

    void scan_callback(const LaserScan::ConstPtr &scan_msg);
    void publish_scans();
    void angle_segment_2d(const boost::shared_ptr<vector<vector<int>>> &output);
    std::vector<int> label_component_bfs(int i, int label, std::vector<int> &labels);
    bool angle_test(unsigned int i);

  protected:
    int angle_threshold;
    std::vector<LaserScan::Ptr> segmented_scans;

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub;

    std::vector<ros::Publisher> scan_pubs;

    sensor_msgs::LaserScan::ConstPtr scan_msg_;
    LaserScan::Ptr leer_scan;
};
}

#endif