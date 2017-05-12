#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>

#include <laser_geometry.h>

class LaserToCloud3d
{

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

  public:
    LaserToCloud3d(ros::NodeHandle *n, double freq = 0.0, const std::string &fixed_frame)
        : nh(*n), fixed_frame(fixed_frame), freq(freq)
    {
        ROS_INFO("LaserToCloud3d constructed");
        //invalid points are goint to be NaN (x<min || x>max)
        invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
        invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
        invalid_point_.z = std::numeric_limits<float>::quiet_NaN();

        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_to_cloud_3d", 1);
        scan_sub = nh.subscribe("scan", 1, &LaserToCloud3d::scanCallback, this);
    }

    ~LaserToCloud3d()
    {
        ROS_INFO("LaserToCloud3d destructed");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform(fixed_frame, scan_in->header.frame_id,
                                                        ros::Time::now(), ros::Duration(3));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        projector_.transformLaserScanToPointCloud(fixed_frame, *scan_in,
                                                  cloud_msg, tfListener);
        if (!freq)
        {
            if rotation of scan_in->header.frame_id to fixed_frame >360
                from pcl to cloud_msg
            else 
                create cloud from scan_in
                pcl.add(cloud)
                return
        }
        else
        {
            if (scan_in->header.stamp - lasttime > 1/freq)
                from pcl to cloud_msg
            else
                create cloud from scan_in
                pcl.add(cloud)
                return
        }
        pub.publish(cloud_msg);
    }

  protected:
    ros::NodeHandle nh;
    ros::Publisher cloud_pub;
    ros::Subscriber scan_sub;

    laser_geometry::LaserProjection projector_;

    sensor_msgs::PointCloud2 cloud_msg;
    tf2_ros::tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::string fixed_frame;
    double freq;
};