#include <angle_segment_3d/angle_segment_3d_viz.h>
#include <cmath>

namespace anglesegment
{

#define PI 3.14159265358979323846

AngleSegment3dViz::AngleSegment3dViz(ros::NodeHandle *n, int angle_threshold) : nh_(*n), angle_threshold(angle_threshold)
{
    ROS_INFO("LaserToCloud constructed");
    // prev_scan_msg_ = new LaserScan;
    scan_pub = nh_.advertise<LaserScan>("/segmented_scan", 1);
    scan_sub = nh_.subscribe("/hokuyo/scan/raw", 1, &AngleSegment3dViz::scan_callback, this);
}

void AngleSegment3dViz::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    scan3d.push(scan_msg);
}

bool AngleSegment3dViz::angle_test(const LaserScan::Ptr &scan_msg, unsigned int i)
{
    float rmax, rmin, alpha;
    alpha = scan_msg->angle_increment; //radian
    if (scan_msg->ranges[i] > scan_msg->ranges[i - 1])
    {
        rmax = scan_msg->ranges[i];
        rmin = scan_msg->ranges[i - 1];
    }
    else
    {
        rmax = scan_msg->ranges[i - 1];
        rmin = scan_msg->ranges[i];
    }
    float artan = atan(rmin * sin(alpha) / (rmax - rmin * cos(alpha)));
    float beta = 180 * artan / PI;
    //ROS_INFO("rmin %f, rmax %f, alpha %f, beta %f", rmin, rmax, alpha, beta);
    return beta > angle_threshold;
}

bool AngleSegment3dViz::angle_test(const LaserScan::Ptr &scan_msg, unsigned int i, int alpha)
{
    float rmax, rmin, alpha;
    alpha = scan_msg->angle_increment; //radian
    if (scan_msg->ranges[i] > scan_msg->ranges[i - 1])
    {
        rmax = scan_msg->ranges[i];
        rmin = scan_msg->ranges[i - 1];
    }
    else
    {
        rmax = scan_msg->ranges[i - 1];
        rmin = scan_msg->ranges[i];
    }
    float artan = atan(rmin * sin(alpha) / (rmax - rmin * cos(alpha)));
    float beta = 180 * artan / PI;
    //ROS_INFO("rmin %f, rmax %f, alpha %f, beta %f", rmin, rmax, alpha, beta);
    return beta > angle_threshold;
}

} // anglesegment namespace