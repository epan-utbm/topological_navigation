// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "topological_navigation/TopologicalPath.h"
#include "topological_navigation/PointArray.h"

int main(int argc, char** argv) {
  ros::init(argc, argv,"send_data");
  ros::NodeHandle nh;


  topological_navigation::TopologicalPath::Request req;
  topological_navigation::TopologicalPath::Response res;

  topological_navigation::PointArray waypoints;

  ros::ServiceClient client = nh.serviceClient<topological_navigation::TopologicalPath>("topological_path");
  
  geometry_msgs::Point starting, ending;
  geometry_msgs::Point waypoints0,waypoints1;

  starting.x = 10;  starting.y = 10;    starting.z = 0;
  ending.x = 50;    ending.y = 50;      ending.z = 0;

  waypoints0.x = 20;  waypoints0.y = 10;    waypoints0.z = 0;
  waypoints1.x = 15;  waypoints1.y = 15;    waypoints1.z = 0;

  waypoints.points.push_back(waypoints0);
  waypoints.points.push_back(waypoints1);

  req.starting = starting;
  req.ending = ending;
  req.waypoints = waypoints;

  if(client.call(req,res)){
    ROS_INFO("Topological_navigation get all points");
  }
  else
  {
    ROS_ERROR("Fail to call service topological_path");
    ROS_ERROR("No path found");
    return false;
  }

  return 0;
}
