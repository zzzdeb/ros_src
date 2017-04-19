#include <angle_segment_2d/angle_segment_2d.h>

namespace anglesegment{


AngleSegment2d::AngleSegment2d(ros::NodeHandle* n):nh_(*n){
            ROS_INFO("LaserToCloud constructed");

            scan_pub = nh_.advertise<LaserScan>("segmented_scan",1);
            scan_sub = nh_.subscribe("scan", 1, &AngleSegment2d::scan_callback, this);
        }
void AngleSegment2d::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            LaserScan::ConstPtr output(new LaserScan(*scan_msg));
        
            // for (unsigned int i = 0; i < output->ranges.size(); ++i)
            // {
            //     PointT& p = cloud->points[i];
            //     float range = scan_msg->ranges[i];
            //     if (range > scan_msg->range_min && range < scan_msg->range_max)
            //     {
            //         float angle = scan_msg->angle_min + i*scan_msg->angle_increment;

            //         p.x = range * cos(angle);
            //         p.y = range * sin(angle);
            //         p.z = 0.0;
            //     }
            //     else
            //         p = invalid_point_;
            // }

            scan_pub.publish(output);
        }

    } // anglesegment namespace