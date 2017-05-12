// #include "angle_segment_3d/angle_segmenter.h"
#include <ros/ros.h>
#include <string>

#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <Eigen/

// #include <pcl/

class RosAngleSegment
{
    typedef sensor_msgs::PointCloud2 Cloud2;
    typedef pcl::PointWithRange PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

  public:
    RosAngleSegment(ros::NodeHandle nh, ros::NodeHandle nh_p) : nh_(nh), nh_p_(nh_p)
    {

        cloud_sub = nh_.subscribe("/assembled_laser", 10, &RosAngleSegment::callback, this);
        cloud_pub = nh_.advertise<Cloud2>("rangedCloud", 1);
    }

    void callback(const Cloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(input, cloud);
        pcl::RangeImage image;
        float res_x = pcl::deg2rad(4.5);
        float res_y = pcl::deg2rad(180.0 / 793.0); //falsch muss aus laser gelesen werden
        // We now want to create a range image from the above point cloud, with a 1deg angular resolution
        float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));   // 360.0 degree in radians
        float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
        Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
        float noiseLevel = 0.00;
        float minRange = 0.0f;
        int borderSize = 1;

        pcl::RangeImage rangeImage;
        image.createFromPointCloud(cloud, res_x, res_y, maxAngleWidth, maxAngleHeight,
                                        sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

        Cloud2 cloud2;
        pcl::toROSMsg(image, cloud2);
        cloud2.header = input.header;
        cloud_pub.publish(cloud2);
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_;
    ros::Publisher cloud_pub;
    ros::Subscriber cloud_sub;
};

int main(int c, char **v)
{
    ros::init(c, v, "fast_angle_segment");
    ros::NodeHandle nh_p("~");
    ros::NodeHandle nh;
    RosAngleSegment ras(nh, nh_p);
    ros::spin();
    return 0;
}
