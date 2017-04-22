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

#define PI 3.14159265358979323846

AngleSegment2dPub::AngleSegment2dPub(ros::NodeHandle *n, int angle_threshold) : nh_(*n), angle_threshold(angle_threshold)
{

    scan_sub = nh_.subscribe("/front/scan", 1, &AngleSegment2dPub::scan_callback, this);

    leer_scan = LaserScan::Ptr(new LaserScan());
    scan_msg_ = LaserScan::ConstPtr(new LaserScan());

    ROS_INFO("LaserToCloud constructed");
}

void AngleSegment2dPub::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    scan_msg_ = scan_msg;
    leer_scan->header = scan_msg_->header;
    boost::shared_ptr<vector<vector<int>>> labels(new vector<vector<int>>);
    angle_segment_2d(labels);
    ///Debug
    // labels->clear();
    // labels.push_back(vector<int>(1,1));
    for (int i = 0; i < labels->size(); i++)
    {
        LaserScan::Ptr obj = LaserScan::Ptr(new LaserScan());
        obj->header = scan_msg_->header;
        obj->angle_min = scan_msg_->angle_min;
        obj->angle_max = scan_msg_->angle_max;
        obj->angle_increment = scan_msg_->angle_increment;
        obj->time_increment = scan_msg_->time_increment;
        obj->scan_time = scan_msg_->scan_time;
        obj->range_min = scan_msg_->range_min;
        obj->range_max = scan_msg_->range_max;
        obj->ranges.resize((*labels)[i].size());
        for (int j = 0; j < obj->ranges.size(); j++)
            obj->ranges[j] = scan_msg_->ranges[(*labels)[i][j]];
        segmented_scans.push_back(obj);
    }
    publish_scans();
    ROS_INFO("%i segmentation", labels->size());
    segmented_scans.clear();
}

void AngleSegment2dPub::publish_scans()
{
    if (scan_pubs.size() < segmented_scans.size())
    {
        scan_pubs.resize(segmented_scans.size());
        for (unsigned int i = 0; i < scan_pubs.size(); i++)
        {
            std::ostringstream out;
            out << "object " << i + 1;
            scan_pubs[i] = nh_.advertise<LaserScan>("a" + std::to_string(i), 1);
        }
    } // resizing

    for (unsigned int i = 0; i < segmented_scans.size(); i++)
    {
        scan_pubs[i].publish(segmented_scans[i]);
    } // publishing

    for (unsigned int i = segmented_scans.size(); i < scan_pubs.size(); i++)
    {
        scan_pubs[i].publish(leer_scan);
    }
}

void AngleSegment2dPub::angle_segment_2d(const boost::shared_ptr<vector<vector<int>>> &output)
{
    //vector<vector<int> > output;
    vector<int> labels(scan_msg_->ranges.size(), 0);
    int label = 1;
    for (unsigned int i = 0; i < labels.size(); i++) // only one loop because of one laser scan;
    {
        if (labels[i] == 0)
        {
            //ROS_INFO("angle_segment_2d");
            vector<int> temp = label_component_bfs(i, label, labels);
            if (temp.size() > 5)
                output->push_back(temp);
            label++;
        }
    }
    //return output;
}

vector<int> AngleSegment2dPub::label_component_bfs(int i, int label, vector<int> &labels)
{
    queue<int> l_queue;
    l_queue.push(i);
    vector<int> output;

    while (!l_queue.empty())
    {
        i = l_queue.front();
        labels[i] = label;
        output.push_back(i);
        //for neighboars
        if (angle_test(i + 1))
            l_queue.push(i + 1);
        l_queue.pop();
    }
    return output;
}

bool AngleSegment2dPub::angle_test(unsigned int i)
{
    if (i == scan_msg_->ranges.size())
        return false; // out of range
    float rmax, rmin, alpha;
    alpha = scan_msg_->angle_increment * 180 / PI;
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

//   /**
//    * @brief      Calculates the labels running over the whole image.
//    */
//   void ComputeLabels(DiffFactory::DiffType diff_type) override {
//     _label_image =
//         cv::Mat::zeros(_depth_image_ptr->size(), cv::DataType<uint16_t>::type);
//     auto diff_helper_ptr =
//         DiffFactory::Build(diff_type, _depth_image_ptr, &_params);
//     // initialize the label
//     uint16_t label = 1;

//     for (int row = 0; row < _label_image.rows; ++row) {
//       for (int col = 0; col < _label_image.cols; ++col) {
//         if (_label_image.at<uint16_t>(row, col) > 0) {
//           // we have already labeled this point
//           continue;
//         }
//         if (_depth_image_ptr->at<float>(row, col) < 0.001f) {
//           // depth is zero, not interested
//           continue;
//         }
//         LabelOneComponent(label, PixelCoord(row, col), diff_helper_ptr.get());
//         // we have finished labeling this connected component. We now need to
//         // label the next one, so we increment the label
//         label++;
//       }
//     }
//   }

// /**
//    * @brief      Label a single connected component with BFS. Can be done faster
//    *             if we augment the queue with a hash or smth.
//    *
//    * @param[in]  label        Label this component with this label
//    * @param[in]  start        Start pixel
//    * @param[in]  diff_helper  The difference helper
// */
// void LabelOneComponent(uint16_t label, const PixelCoord& start,
//                      const AbstractDiff* diff_helper) {
//     // breadth first search
//     std::queue<PixelCoord> labeling_queue;
//     labeling_queue.push(start);
//     // while the queue is not empty continue removing front point adding its
//     // neighbors back to the queue - breadth-first-search one component
//     size_t max_queue_size = 0;
//     while (!labeling_queue.empty()) {
//     max_queue_size = std::max(labeling_queue.size(), max_queue_size);
//     // copy the current coordinate
//     const PixelCoord current = labeling_queue.front();
//     labeling_queue.pop();
//     uint16_t current_label = LabelAt(current);
//     if (current_label > 0) {
//         // we have already labeled this point. No need to add it.
//         continue;
//     }
//     // set the label of this point to current label
//     SetLabel(current, label);

//     // check the depth
//     auto current_depth = DepthAt(current);
//     if (current_depth < 0.001f) {
//         // depth of this point is wrong, so don't bother adding it to queue
//         continue;
//     }
//     for (const auto& step : Neighborhood) {
//         PixelCoord neighbor = current + step;
//         if (neighbor.row < 0 || neighbor.row >= _label_image.rows) {
//         // point doesn't fit
//         continue;
//         }
//         // if we just went over the borders in horiz direction - wrap around
//         neighbor.col = WrapCols(neighbor.col);
//         uint16_t neigh_label = LabelAt(neighbor);
//         if (neigh_label > 0) {
//         // we have already labeled this one
//         continue;
//         }
//         auto diff = diff_helper->DiffAt(current, neighbor);
//         if (diff_helper->SatisfiesThreshold(diff, _radians_threshold)) {
//         labeling_queue.push(neighbor);
//         }
//     }
//     }
// }