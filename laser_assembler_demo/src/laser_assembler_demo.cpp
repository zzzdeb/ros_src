#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>

// #include <ros_graph/Clock.h>

using namespace laser_assembler;

// ros::Time timeabc;

// void callback(ros_graph::Clock clock){
//   timeabc = 
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  ros::NodeHandle n;
  ros::service::waitForService("assemble_scans2");
  ros::ServiceClient client = n.serviceClient<AssembleScans2>("assemble_scans2");
  AssembleScans2 srv;
  srv.request.begin = ros::Time(0);
  //srv.request.begin = ros::Time::now() - ros::Duration(5);
  srv.request.end = ros::Time::now();

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("assembled_laser", 1);
  // ros::Subscriber sum = n.subscribe("clock", callback, 10);
  ros::Rate hz(0.5);
  while (ros::ok())
  {
    if (client.call(srv))
    {
      
      // printf("Got cloud with %u points\n", srv.response.cloud.points.size());
      //srv.request.begin = ros::Time::now() - ros::Duration(1);
      srv.request.begin = srv.request.end;
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