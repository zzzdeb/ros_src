#include <angle_segment_2d/angle_segment_2d.h>

namespace anglesegment{


AngleSegment2d::AngleSegment2d(ros::NodeHandle* n):nh_(*n){
            ROS_INFO("LaserToCloud constructed");
            //invalid points are goint to be NaN (x<min || x>max)
            invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
            invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
            invalid_point_.z = std::numeric_limits<float>::quiet_NaN();
            
            cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("laser_to_cloud",1);
            scan_sub = nh_.subscribe("scan", 1, &AngleSegment2d::callback, this);
        }
void AngleSegment2d::callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
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

    } // anglesegment namespace