#include <angle_segment_2d/angle_segment_2d_viz.h>
#include <cmath>

namespace anglesegment{

#define PI 3.14159265358979323846

AngleSegment2dViz::AngleSegment2dViz(ros::NodeHandle* n, int angle_threshold):nh_(*n), angle_threshold(angle_threshold){
            ROS_INFO("LaserToCloud constructed");

            scan_pub = nh_.advertise<LaserScan>("/segmented_scan",1);
            scan_sub = nh_.subscribe("/front/scan", 1, &AngleSegment2dViz::scan_callback, this);
        }



void AngleSegment2dViz::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            LaserScan::Ptr output(new LaserScan(*scan_msg));
            int label = 1;
            int current_label_begin = 0;
            int factor = 10;
            int notanobject = 1;
            int currentlabel;
            output->intensities[0] = factor;
            for (unsigned int i = 1; i < output->ranges.size(); ++i)
            {
                float range = output->ranges[i];
                if (range > scan_msg->range_min && range < scan_msg->range_max)
                {
                    if (!angle_test(output, i)){
                        if (i-current_label_begin>5) //mindest punkte
                        {
                            currentlabel=label*factor;
                            label++;
                        }
                        else
                            currentlabel = notanobject;
                        for (int j=current_label_begin; j<i; j++)
                            if(output->intensities[j]!=notanobject)
                                output->intensities[j]=currentlabel ;
                        current_label_begin = i;
                    }
                }
                else
                    output->intensities[i] = notanobject;
            }
            ROS_INFO("%i Object segmented", label);
            scan_pub.publish(output);
        }

bool AngleSegment2dViz::angle_test(const LaserScan::Ptr& scan_msg, unsigned int i)
{
    float rmax, rmin, alpha;
    alpha = scan_msg->angle_increment; //radian
    if (scan_msg->ranges[i]>scan_msg->ranges[i-1]){
        rmax = scan_msg->ranges[i];
        rmin = scan_msg->ranges[i-1];
    }
    else{ 
        rmax = scan_msg->ranges[i-1];
        rmin = scan_msg->ranges[i];
    }
    float artan = atan(rmin*sin(alpha)/(rmax-rmin*cos(alpha)));
    float beta = 180*artan/PI;
    //ROS_INFO("rmin %f, rmax %f, alpha %f, beta %f", rmin, rmax, alpha, beta);  
    return beta>angle_threshold;
}

} // anglesegment namespace