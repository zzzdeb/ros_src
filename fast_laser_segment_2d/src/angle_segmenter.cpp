#include "include/angle_segmenter.h"

#include <tuple>
#include <queue>
#include <pcl/range_image/range_image.h>

namespace angle_segmenter
{

typedef std::tuple<int,int> PixelCoord;

AngleSegmenter(const pcl::PCLPointCloud2Ptr cloud, int angle_threshold)
    : angle_threshold(angle_threshold)
{
    float res_x = pcl::deg2rad(4.5);
    float res_y = pcl::deg2rad(180.0 / 793.0); //falsch muss aus laser gelesen werden
    rcloud->createFromPointCloud(*cloud, angular_resolution_x_ = res_x, angular_resolution_y_ = reg_y);
}

~AngleSegmenter()
{
    ROS_INFO("AngleSegmenter destructed");
}
void segment()
{

}

  /**
   * @brief      Label a single connected component with BFS. Can be done faster
   *             if we augment the queue with a hash or smth.
   *
   * @param[in]  label        Label this component with this label
   * @param[in]  start        Start pixel
   * @param[in]  diff_helper  The difference helper
   */
  void LabelOneComponent(uint16_t label, const PixelCoord& start,  string difftype="angle") {
    // breadth first search
    std::queue<PixelCoord> labeling_queue;
    labeling_queue.push(start);
    // while the queue is not empty continue removing front point adding its
    // neighbors back to the queue - breadth-first-search one component
    size_t max_queue_size = 0;
    while (!labeling_queue.empty()) {
      max_queue_size = std::max(labeling_queue.size(), max_queue_size);
      // copy the current coordinate
      const PixelCoord current = labeling_queue.front();
      labeling_queue.pop();
      uint16_t current_label = LabelAt(current);
      if (current_label > 0) {
        // we have already labeled this point. No need to add it.
        continue;
      }
      // set the label of this point to current label
      SetLabel(current, label);

      // check the depth
      auto current_depth = DepthAt(current);
      if (current_depth < 0.001f) {
        // depth of this point is wrong, so don't bother adding it to queue
        continue;
      }
      for (const auto& step : Neighborhood) {
        PixelCoord neighbor = current + step;
        if (neighbor.row < 0 || neighbor.row >= _label_image.rows) {
          // point doesn't fit
          continue;
        }
        // if we just went over the borders in horiz direction - wrap around
        neighbor.col = WrapCols(neighbor.col);
        uint16_t neigh_label = LabelAt(neighbor);
        if (neigh_label > 0) {
          // we have already labeled this one
          continue;
        }
        auto diff = diff_helper->DiffAt(current, neighbor);
        if (diff_helper->SatisfiesThreshold(diff, _radians_threshold)) {
          labeling_queue.push(neighbor);
        }
      }
    }
  }

bool angle_test(const LaserScan::Ptr &scan_msg, unsigned int i);

}