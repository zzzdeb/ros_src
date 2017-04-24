#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/PointCloud.h>

using namespace laser_assembler;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  ros::NodeHandle n;
  ros::service::waitForService("assemble_scans");
  ros::ServiceClient client = n.serviceClient<AssembleScans>("assemble_scans");
  AssembleScans srv;
  srv.request.begin = ros::Time(0.0);
  //srv.request.begin = ros::Time::now() - ros::Duration(5);
  srv.request.end = ros::Time::now();

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("assembled_laser", 1);
  ros::Rate hz(1);
  while (ros::ok())
  {
    if (client.call(srv))
    {
      printf("Got cloud with %u points\n", srv.response.cloud.points.size());
      //srv.request.begin = ros::Time::now() - ros::Duration(1);
      srv.request.begin = ros::Time(0.0);
      srv.request.end = ros::Time::now();
      pub.publish(srv.response.cloud);
      ros::spinOnce();
      hz.sleep();
    }
    else
      printf("Service call failed\n");
  }
  return 0;
}