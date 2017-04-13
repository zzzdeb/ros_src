#include "laser_to_cloud/laser_to_cloud.h"

int main(int c, char** v){
    
    ros::init(c, v, "laserToCloud");
    ros::NodeHandle n("~");
    LaserToCloud nh(&n);
    ros::spin();
    return 0;
}