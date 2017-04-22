#include "angle_segment_2d/angle_segment_2d_viz.h"
#include "angle_segment_2d/angle_segment_2d_pub.h"
#include "tclap/CmdLine.h"
#include <string>
#include <ros/ros.h>

int main(int c, char** v){
    TCLAP::CmdLine cmd(
        "Subscribe to laserscan and segment it.",' ', "1.0"); // explanation, dilimiter, version

    TCLAP::ValueArg<int> angle_arg(
      "", "angle",
      "Threshold angle. Below this value, the objects are separated", false, 10,
      "int");
    TCLAP::ValueArg<int> mode(
        "", "mode",
        "Vizualisieren = 0; Laserscans erstellen = 1", false, 0, "int");  //

    cmd.add(mode);
    cmd.add(angle_arg);
    cmd.parse(c, v);

    ros::init(c, v, "fast_angle_segment");
    ros::NodeHandle nh("~");
    int angle_threshold = angle_arg.getValue();
    anglesegment::AngleSegment2dPub* nh_pub;
    anglesegment::AngleSegment2dViz* nh_viz;
    switch(mode.getValue()){
        case 0:
            nh_viz = new anglesegment::AngleSegment2dViz(&nh, angle_threshold);
        break;
        case 1:
            nh_pub = new anglesegment::AngleSegment2dPub(&nh, angle_threshold);
        break;
    }
    ros::spin();
    delete nh_pub, nh_viz;
    return 0;
}
