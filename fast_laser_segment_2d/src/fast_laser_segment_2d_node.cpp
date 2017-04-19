#include "angle_segment_2d/angle_segment_2d.h"

int main(int c, char** v){
    
    ros::init(c, v, "fast_angle_segment");
    ros::NodeHandle nh("~");
    AngleSegment2d nh(&n);
    ros::spin();
    return 0;
}