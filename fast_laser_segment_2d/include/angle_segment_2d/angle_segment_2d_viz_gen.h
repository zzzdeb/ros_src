#ifndef _ANGLE_SEGMENT_2D_VIZ_GEN_
#define _ANGLE_SEGMENT_2D_VIZ_GEN_

#include "angle_segment_2d/angle_segment_2d_pub.h"

namespace anglesegment
{
class AngleSegment2dVizGen : public AngleSegment2dPub
{
  public:
    AngleSegment2dVizGen(ros::NodeHandle *n, int angle_threshold) : AngleSegment2dPub(n, angle_threshold) {}
    ~AngleSegment2dVizGen()
    {
        ROS_INFO("AngleSegment2dPubGen destructed");
    }
    void scan_callback(const LaserScan::ConstPtr &scan_msg)
{
    scan_msg_ = scan_msg;
    leer_scan->header = scan_msg_->header;
    boost::shared_ptr<vector<ViPtr>> labelptr(new vector<ViPtr>);
    angle_segment_2d(labelptr);
    ///Debug
    // labels->clear();
    // labels.push_back(vector<int>(1,1));
    for (int i = 0; i < labelptr->size(); i++)
    {
        LaserScan::Ptr obj = LaserScan::Ptr(new LaserScan());
        obj->header = scan_msg_->header;
        obj->angle_min = scan_msg_->angle_min+scan_msg_->angle_increment*(labelptr->at(i)->front());
        obj->angle_max = scan_msg_->angle_max+scan_msg_->angle_increment*(labelptr->at(i)->back());
        obj->angle_increment = scan_msg_->angle_increment;
        obj->time_increment = scan_msg_->time_increment;
        obj->scan_time = scan_msg_->scan_time;
        obj->range_min = scan_msg_->range_min;
        obj->range_max = scan_msg_->range_max;
        obj->ranges.resize(labelptr->at(i)->size());
        for (int j = 0; j < obj->ranges.size(); j++)
            obj->ranges[j] = scan_msg_->ranges[(*labelptr)[i]->at(j)];
        segmented_scans.push_back(obj);
    }
    publish_scans();
    ROS_INFO("%i blabla", labelptr->size());
    segmented_scans.clear();
}
  protected:
};
}

#endif