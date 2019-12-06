// ROS
#include <ros/ros.h>
#include "topological_navigation/TopologicalPath.h"
#include "topological_navigation/PointArray.h"
#include <geometry_msgs/Point.h>


int main(int argc, char** argv) {
  ros::init(argc, argv,"send_data");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::serviceClient client = private_nh.serviceClient<topological_navigation::TopologicalPath>("topological_path");
  topological_navigation::TopologicalPath srv;

  geometry_msgs::Points starting, ending;

  starting.x = 10;  starting.y = 10;    starting.z = 10;
  ending.x = 30;    ending.y = 30;      ending.z = 30;

  srv.request.starting = starting;
  srv.request.ending = ending;
  //we also have to send a points list 

  if(client.call(srv)){
      ROS_INFO("Topological_navigation get points");
  }
  else
  {
      ROS_ERROR("Fail to call service get_points");
      return 1;
  }

  return 0;
}