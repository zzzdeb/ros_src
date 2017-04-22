#include "angle_segment_2d/angle_segment_2d.h"
#include <ros/ros.h>

int main(int c, char** v){
    
    ros::init(c, v, "fast_angle_segment");
    ros::NodeHandle nh("~");
    int angle_threshold = 10;
    anglesegment::AngleSegment2d segmentation(&nh, angle_threshold);
    ros::spin();
    return 0;
}
