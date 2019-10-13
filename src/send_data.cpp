//This CPP file is only an example to show you how to interact with the topological_navigation package if you don't use you can delete it.

//ROS
#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/Point.h>

//Rosservice
#include "topological_navigation/SE_points.h"
#include "topological_navigation/Path.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_data");

  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<topological_navigation::SE_points>("get_points");
  ros::ServiceClient path_client = nh.serviceClient<topological_navigation::Path>("get_path");
  topological_navigation::SE_points srv;
  topological_navigation::Path path_srv;

  //creating the starting and ending point
  geometry_msgs::Point starting, ending;

  starting.x = 10; starting.y = 10; starting.z = 0;
  ending.x = 100;  ending.y = 10;   ending.z = 0;

  //set the request
  srv.request.starting = starting;
  srv.request.ending = ending;

  //call the service get_points
  if (client.call(srv))
  {
    ROS_INFO("Topological_navigation get points");
  }
  else
  {
    ROS_ERROR("Failed to call service get_points");
    return 1;
  }

  //call the service get_path
  if(path_client.call(path_srv)){
    ROS_INFO("The path has been received");//here you get an array of integer 
  }
  else
  {
    ROS_ERROR("Failed to call the service get_path");
  }
  

  return 0;
}
