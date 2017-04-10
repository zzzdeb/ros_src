#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <iostream>


class ScanToCloud{

    typedef pcl::PointXYZ           PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    public:
        ScanToCloud(ros::NodeHandle n) {
            ROS_INFO("LaserToCloud constructed");
            //invalid points are goint to be NaN (x<min || x>max)
            invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
            invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
            invalid_point_.z = std::numeric_limits<float>::quiet_NaN();
            
            cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_to_cloud",1);
            //std::string scan_topic;
            //std::cout<<nh.getParam("scan_topic", scan_topic)<<"aaaaaaaaaaaaaaaa";
            scan_sub = nh.subscribe("front/scan", 1, &ScanToCloud::callback, this);
        }

        ~ScanToCloud(){
            ROS_INFO("LaserToCloud destructed");
        }

        void callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            sensor_msgs::PointCloud2 output;
            PointCloudT::Ptr cloud(new PointCloudT);
            cloud->points.resize(scan_msg->ranges.size());
            for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
            {
                PointT& p = cloud->points[i];
                float range = scan_msg->ranges[i];
                if (range > scan_msg->range_min && range < scan_msg->range_max)
                {
                    float angle = scan_msg->angle_min + i*scan_msg->angle_increment;

                    p.x = range * cos(angle);
                    p.y = range * sin(angle);
                    p.z = 0.0;
                }
                else
                    p = invalid_point_;
            }

            cloud->width = scan_msg->ranges.size();
            cloud->height = 1;
            cloud->is_dense = false; //contains nans
            pcl_conversions::toPCL(scan_msg->header, cloud->header);
            pcl::toROSMsg(*cloud, output);
            cloud_pub.publish(output);
        }

    protected:
        ros::NodeHandle nh;
        ros::Publisher cloud_pub;
        ros::Subscriber scan_sub;

        PointT invalid_point_;
};