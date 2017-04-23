#include <angle_segment_2d/angle_segment_2d_pub.h>
#include <cmath>
#include <vector>
#include <string>
#include <queue>
#include <sstream>
#include <boost/shared_ptr.hpp>

using namespace std;

namespace anglesegment
{
typedef boost::shared_ptr<vector<int>> ViPtr;
#define PI 3.14159265358979323846

AngleSegment2dPub::AngleSegment2dPub(ros::NodeHandle *n, int angle_threshold) : nh_(*n), angle_threshold(angle_threshold)
{

    scan_sub = nh_.subscribe("/hokuyo/scan/raw", 1, &AngleSegment2dPub::scan_callback, this);

    leer_scan = LaserScan::Ptr(new LaserScan());
    scan_msg_ = LaserScan::ConstPtr(new LaserScan());

    ROS_INFO("LaserToCloud constructed");
}

void AngleSegment2dPub::scan_callback(const LaserScan::ConstPtr &scan_msg)
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
    ROS_INFO("%i segmentation", labelptr->size());
    segmented_scans.clear();
}

void AngleSegment2dPub::publish_scans()
{
    if (scan_pubs.size() < segmented_scans.size())
    {
        scan_pubs.resize(segmented_scans.size());
        for (unsigned int i = 0; i < scan_pubs.size(); i++)
        {
            scan_pubs[i] = nh_.advertise<LaserScan>('S' + std::to_string(i), 1);
        }
    } // resizing

    if (segmented_scans.size() < scan_pubs.size() - 5)
    {
        scan_pubs.resize(segmented_scans.size());
        for (unsigned int i = 0; i < scan_pubs.size(); i++)
        {
            scan_pubs[i] = nh_.advertise<LaserScan>('S' + std::to_string(i), 1);
        }
    }

    for (unsigned int i = 0; i < segmented_scans.size(); i++)
    {
        scan_pubs[i].publish(segmented_scans[i]);
    } // publishing

    for (unsigned int i = segmented_scans.size(); i < scan_pubs.size(); i++)
    {
        scan_pubs[i].publish(leer_scan);
    }
}

void AngleSegment2dPub::angle_segment_2d(const boost::shared_ptr<vector<ViPtr>> &labelptr)
{
    //vector<vector<int> > output;
    vector<int> labels(scan_msg_->ranges.size());
    int label = 1;
    for (unsigned int i = 0; i < labels.size(); i++) // only one loop because of one laser scan;
    {
        if (labels[i] == 0)
        {
            //ROS_INFO("angle_segment_2d");
            ViPtr temp = label_component_bfs(i, label, labels);
            if (temp->size() > 5)
                labelptr->push_back(temp);
            label++;
        }
    }
    //return output;
}

ViPtr AngleSegment2dPub::label_component_bfs(int i, int label, vector<int> &labels)
{
    queue<int> l_queue;
    l_queue.push(i);
    ViPtr output(new vector<int>);

    while (!l_queue.empty())
    {
        i = l_queue.front();
        l_queue.pop();
        if (labels[i]!=0) continue; //labeled
        if (scan_msg_->ranges[i] > scan_msg_->range_max && scan_msg_->ranges[i] < scan_msg_->range_min)
        {
            labels[i] = -1; // invalid points
            continue;
        }
        labels[i] = label;
        output->push_back(i);
        //for neighboars
        if (angle_test(i + 1))
            l_queue.push(i + 1);
    }
    return output;
}

bool AngleSegment2dPub::angle_test(unsigned int i)
{
    if (i == scan_msg_->ranges.size())
        return false; // out of range
    float rmax, rmin, alpha;
    alpha = scan_msg_->angle_increment;
    if (scan_msg_->ranges[i] > scan_msg_->ranges[i - 1])
    {
        rmax = scan_msg_->ranges[i];
        rmin = scan_msg_->ranges[i - 1];
    }
    else
    {
        rmax = scan_msg_->ranges[i - 1];
        rmin = scan_msg_->ranges[i];
    }
    float beta = 180 * atan(rmin * sin(alpha) / (rmax - rmin * (cos(alpha)))) / PI;
    //ROS_INFO("rmin %f, rmax %f, alpha %f, beta %f", rmin, rmax, alpha, beta);
    return beta > angle_threshold;
}

} // anglesegment namespace

