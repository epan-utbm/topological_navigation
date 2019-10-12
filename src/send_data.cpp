#include "ros/ros.h"
#include "topological_navigation/SE_points.h"
#include "topological_navigation/Path.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_data");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<topological_navigation::SE_points>("get_points");
  ros::ServiceClient path_client = nh.serviceClient<topological_navigation::Path>("get_path");
  topological_navigation::SE_points srv;
  topological_navigation::Path path_srv;

  geometry_msgs::Point starting, ending;

  starting.x = 10; starting.y = 10; starting.z = 0;
  ending.x = 100;  ending.y = 10;   ending.z = 0;


  srv.request.starting = starting;
  srv.request.ending = ending;

  if (client.call(srv))
  {
    ROS_INFO("Topological_navigation get points");
  }
  else
  {
    ROS_ERROR("Failed to call service get_points");
    return 1;
  }

  if(path_client.call(path_srv)){
    ROS_INFO("The path has been received");
  }
  else
  {
    ROS_ERROR("Failed to call the service get_path");
  }
  

  return 0;
}
