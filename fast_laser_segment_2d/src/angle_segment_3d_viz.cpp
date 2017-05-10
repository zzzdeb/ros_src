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


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        ros::Time now = ros::Time::now();
        // transformStamped = tfBuffer.lookupTransform("platform_laser_rotation/base/rotation_center", "platform_laser_rotation/carrier/laser_mount", now,
        //                                             ros::Duration(3.0));
        transformStamped.header = msg->header;
        transformStamped.header.frame_id = "platform_laser_rotation/base/rotation_center";
        transformStamped.child_frame_id = "platform_laser_rotation/carrier/laser_mount";
        transformStamped.transform.translation.x = -0.043;
        transformStamped.transform.translation.y = 0.066;
        transformStamped.transform.translation.z = 0.130;
        
        double dur = (msg->header.stamp-first_scan_time).toSec();
        tf2::Quaternion rot((dur/0.025)*0.0785, 0, -1.570);
        transformStamped.transform.rotation.x = rot.x();
        transformStamped.transform.rotation.y = rot.y();
        transformStamped.transform.rotation.z = rot.z();
        transformStamped.transform.rotation.w = rot.w();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform a to b: %s", ex.what());
    }
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        RichPoint p;
        Eigen::Vector3d p_in;
        Eigen::Vector3d p_out;
        float range = msg->ranges[i];
        if (range > msg->range_min && range < msg->range_max)
        {
            float angle = msg->angle_min + i * msg->angle_increment;

            p_in.x() = range * sin(angle);
            p_in.y() = range * cos(angle);
            p_in.z() = 0;

            tf2::doTransform(p_in, p_out, transformStamped);
            p.AsEigenVector() = p_out.cast<float>();
            cloud.push_back(p);
        }
        // else
        //     p = invalid_point_;
    }

    return make_shared<Cloud>(cloud);


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