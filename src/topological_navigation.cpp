// ROS
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>

// CPP
#include <vector>
#include <yaml-cpp/yaml.h>

// #include <stdbool.h>

#define ABS(a) (((a) < 0) ? -(a) : (a))

using namespace std;

int rows_, cols_;
double mapResolution_;
vector<vector<int> > grid_;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid &map);
void printGrid();

bool getWaypoints(string &filename, vector<geometry_msgs::Point> &waypoints);
int BresenhamPlanner(geometry_msgs::Point &a, geometry_msgs::Point &b); // local path planner
void DijkstraPlanner(int s, vector<vector<int> > &links, vector<int> &path); // global path planner
void queryPath(int s, int e, vector<int> &path);

int main(int argc, char** argv) {
  ros::init(argc, argv,"topological_navigation");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  if(!requestMap(nh)) {
    exit(-1);
  }
  //printGrid();
  
  string waypoint_file;
  private_nh.param<string>("waypoint_file", waypoint_file, "");
  
  geometry_msgs::Point starting, ending;
  vector<geometry_msgs::Point> waypoints;
  
  if(!waypoint_file.empty()) { // static waypoint list, best for the test
    if(!getWaypoints(waypoint_file, waypoints)) {
      exit(-1);
    }
    
    starting.x = 10; starting.y = 10; starting.z = 0;
    ending.x = 100;  ending.y = 10;   ending.z = 0;
  }
  
  // dynamic waypoint list, best for the competition
  // TODO: get starting and ending points, as well as waypoints list via rosservice
  // TODO: send the path (a list of waypoints joins the start and end points) back to the caller (still via rosservice)


  // Use path-finding algorithms to establish connections between waypoints (i.e. costs)
  string path_planning;
  private_nh.param<string>("path_planning", path_planning, "Bresenham");

  // Building a cost matrix for the Dijkstra algorithm
  int distance;

  waypoints.push_back(starting);
  waypoints.push_back(ending);
  
  vector<vector<int> > waypoints_link(waypoints.size());
  for(int i = 0; i < waypoints.size(); i++) {
    /****** Question for Toma ******/
    /* The cost matrix should be symmetrical.
       It saves the computational resource and that's why we need to
       "complement the cost matrix" at the end of this loop
    */
    
    //for(int j = 0; j <= i; j++) {

    /* However, if we calculate the cost for both directions,
       i.e. A -> B and B -> A, We actually didn't get a symmetric matrix, e.g:
       0	37	-	-	17	27	-	7	-	
       37	0	-	-	20	16	-	30	-	
       -	-	0	-	-	-	-	-	-	
       37	-	-	0	20	-	36	30	-	
       17	20	-	-	0	10	-	10	-	
       27	16	-	-	10	0	26	20	-	
       -	-	-	-	-	26	0	-	-	
       7	30	-	-	10	20	-	0	-	
       -	-	-	-	-	-	-	-	0
       So Toma could you help me to find out the pb?
    */
    for(int j = 0; j < waypoints.size(); j++) {
      if(j == i) {
	waypoints_link[i].push_back(0); // 0 is the cost to itself
      } else {
	// TODO: Select different path-finding algorithms by the parameter
	if(path_planning.compare("Bresenham") == 0) {
	  distance = BresenhamPlanner(waypoints[i], waypoints[j]);
	} else if(path_planning.compare("Astar") == 0) {
	  // TODO
	} else if(path_planning.compare("Nathan") == 0) {
	  // TODO
	}
	waypoints_link[i].push_back(distance);
      }
      if(waypoints_link[i][j] == INT_MAX) {
	cerr << "-\t";
      } else {
	cerr << waypoints_link[i][j] << "\t";
      }
    }
    cerr << endl;
  }
  // TODO: complement the cost matrix

  vector<int> path(waypoints.size());
  DijkstraPlanner(waypoints.size()-2, waypoints_link, path);
  queryPath(waypoints.size()-2, waypoints.size()-1, path);
  
  return 0;
}

bool requestMap(ros::NodeHandle &nh) {
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  
  while(!ros::service::waitForService("static_map",ros::Duration(3.0))) {
    ROS_INFO("waiting for service static_map to become available");
  }
  ROS_INFO("Requesting Map ...");
  
  ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");
  
  if(mapClient.call(req, res)) {
    readMap(res.map);
    return true;
  } else {
    ROS_ERROR("Failed to call map service");
    return false;
  }
}

void readMap(const nav_msgs::OccupancyGrid &map) {
  ROS_INFO("Received a %d X %d map @ %.3f m/px\n", map.info.width, map.info.height, map.info.resolution);

  // Map meta data
  rows_ = map.info.height;
  cols_ = map.info.width;
  mapResolution_ = map.info.resolution;

  // Dynamically resize the grid
  grid_.resize(rows_);
  for(int i = 0; i < rows_; i++) {
    grid_[i].resize(cols_);
  }

  // Fill map data
  int currCell = 0;
  for (int i = 0; i < rows_; i++) {
    for(int j = 0; j < cols_ ; j++) {
      if(map.data[currCell] == 0)
	grid_[i][j] = 0; // unoccupied cell
      else
	grid_[i][j] = 1; // occupied (100) or unknown cell (-1)
      currCell++;
    }
  }
}

void printGrid() {
  //printf("Grid map:\n");
  int freeCells = 0;
  for(int i = 0; i < rows_; i++) {
    //printf("row no. %d\n", i);
    for(int j = 0; j < cols_; j++) {
      printf("%d ",grid_[i][j]);
    }
    printf("\n");
  }
}

bool getWaypoints(string &filename, vector<geometry_msgs::Point> &waypoints) {
  if(!filename.empty()) {
    ROS_INFO_STREAM("Loading waypoint file: " << filename);
    YAML::Node node = YAML::LoadFile(filename);
    for(YAML::const_iterator it = node["waypoints"].begin(); it != node["waypoints"].end(); ++it) {
      geometry_msgs::Point p;
      p.x = (*it)["pose"]["position"]["x"].as<double>();
      p.y = (*it)["pose"]["position"]["y"].as<double>();
      p.z = (*it)["pose"]["position"]["z"].as<double>();
      waypoints.push_back(p);
    }
    ROS_INFO_STREAM("Success!");
    return true;
  } else {
    ROS_WARN_STREAM("No waypoint file loaded.");
    return false;
  }
}

// Bresenham's line algorithm (i.e straight line planner) for local path planning
// i.e. path between any two waypoints
int BresenhamPlanner(geometry_msgs::Point &a, geometry_msgs::Point &b) {
  int sx, sy, dx, dy, e, e2;
  int x = a.x, y = a.y, i = 0;
  
  if(a.x < b.x) {sx = 1;} else {sx = -1;}
  if(a.y < b.y) {sy = 1;} else {sy = -1;}
  
  dx = ABS(a.x - b.x);
  dy = ABS(a.y - b.y);
  e = dx - dy;
  
  while(x != b.x || y != b.y) {
    e2 = e * 2;
    if(e2 > -dy) {e -= dy; x += sx;}
    if(e2 <  dx) {e += dx; y += sy;}
    
    if(grid_[y][x] == 1) { // occupied (100) or unknown cell (-1)
      return INT_MAX;
    }
    i++;
  }
  
  return i;
}

/* Dijkstra algorithm for global path planning (i.e. path from starting point to ending point).
 * CONVENTION: the last two elements in the waypoint list are respectively starting and ending points.
 */
void DijkstraPlanner(int s, vector<vector<int> > &links, vector<int> &path) {
  int n = links.size();
  
  vector<bool> visited(n, false);
  vector<int> dist(n, INT_MAX);
  
  for(int i = 0; i < n; i++) {
    path[i] = i;
  }
  
  dist[s] = 0; // starting point
  for(int i = 0; i < n; i++) {
    int u = -1;
    int min = INT_MAX;
    for(int j = 0; j < n; j++) {
      if(!visited[j] && dist[j] < min) {
	u = j;
	min = dist[j];
      }
    }

    if(u == -1) {
      return;
    }
    
    visited[u] = true;
    for(int v = 0; v < n; v++) {
      if(!visited[v] && dist[u] + links[u][v] < dist[v]) {
	dist[v] = dist[u] + links[u][v];
	path[v] = u;
      }
    }
  }
}

void queryPath(int s, int e, vector<int> &path) {
  if (e == s) {
    cerr << s << " ";
    return;
  }
  queryPath(s, path[e], path);
  cerr << e << endl;
}
