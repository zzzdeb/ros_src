#include <angle_segment_2d/angle_segment_2d.h>
#include <cmath>
// #include <utils/radians.h>

namespace anglesegment{

#define PI 3.14159265358979323846

AngleSegment2d::AngleSegment2d(ros::NodeHandle* n, int angle_threshold):nh_(*n), angle_threshold(angle_threshold){
            ROS_INFO("LaserToCloud constructed");

            scan_pub = nh_.advertise<LaserScan>("segmented_scan",1);
            scan_sub = nh_.subscribe("scan", 1, &AngleSegment2d::scan_callback, this);
        }



void AngleSegment2d::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            LaserScan::Ptr output(new LaserScan(*scan_msg));
            int label = 1;
            int factor = 10;
            output->intensities[0] = factor;
            for (unsigned int i = 1; i < output->ranges.size(); ++i)
            {
                float range = output->ranges[i];
                if (range > scan_msg->range_min && range < scan_msg->range_max)
                {
                    if (angle_test(output, i))
                        output->intensities[i]=label*factor;
                    else {
                        label++;
                        output->intensities[i]=label*factor;
                    }
                }
                else
                    output->intensities[i] = 5;
            }
            ROS_INFO("label max: %i", label);
            scan_pub.publish(output);
        }

bool AngleSegment2d::angle_test(const LaserScan::Ptr& scan_msg, unsigned int i)
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
//     void LabelOneComponent(uint16_t label, const PixelCoord& start,
//                          const AbstractDiff* diff_helper) {
//         // breadth first search
//         std::queue<PixelCoord> labeling_queue;
//         labeling_queue.push(start);
//         // while the queue is not empty continue removing front point adding its
//         // neighbors back to the queue - breadth-first-search one component
//         size_t max_queue_size = 0;
//         while (!labeling_queue.empty()) {
//         max_queue_size = std::max(labeling_queue.size(), max_queue_size);
//         // copy the current coordinate
//         const PixelCoord current = labeling_queue.front();
//         labeling_queue.pop();
//         uint16_t current_label = LabelAt(current);
//         if (current_label > 0) {
//             // we have already labeled this point. No need to add it.
//             continue;
//         }
//         // set the label of this point to current label
//         SetLabel(current, label);

//         // check the depth
//         auto current_depth = DepthAt(current);
//         if (current_depth < 0.001f) {
//             // depth of this point is wrong, so don't bother adding it to queue
//             continue;
//         }
//         for (const auto& step : Neighborhood) {
//             PixelCoord neighbor = current + step;
//             if (neighbor.row < 0 || neighbor.row >= _label_image.rows) {
//             // point doesn't fit
//             continue;
//             }
//             // if we just went over the borders in horiz direction - wrap around
//             neighbor.col = WrapCols(neighbor.col);
//             uint16_t neigh_label = LabelAt(neighbor);
//             if (neigh_label > 0) {
//             // we have already labeled this one
//             continue;
//             }
//             auto diff = diff_helper->DiffAt(current, neighbor);
//             if (diff_helper->SatisfiesThreshold(diff, _radians_threshold)) {
//             labeling_queue.push(neighbor);
//             }
//         }
//         }
//     }