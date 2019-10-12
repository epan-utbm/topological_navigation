# A High-level Implementation of Topological Navigation with ROS

### How to launch it ?

To launch the package you have to open 3 terminals:
 1. roscore
 2. roslaunch topological_navigation topological_navigation.launch
 3. rosrun topological_navigation send_data
 
### How it works ?
The goal of this package is to define waypoints in your map via waypoints generator package [https://github.com/epan-utbm/waypoint_generator].

When you are done with that you can move on the topological_navigation package which is a service. 
To make it works you have to send the starting and ending point and the waypoints you have created to it.

After that it will find the shortest path to go from the starting point to the ending via selected waypoints.

For that it needs to find links (i.e. costs) between the waypoints. In other words, is there any obstacles between two waypoints?

To do so you can use the algorithm you want by passing it name in the following line :

```c
    private_nh.param<string>("path_planning", path_planning, "Bresenham");
```

In this case we implemented Bresemham algorithm. 


### Bresemham algorithm 
This algorithm draw an invisible line passing through pixels between two waypoints. If one pixel is occupied (100) or unknown cell (-1), we can conclude that there is a wall.

So we simulate all trajectories possibilities for each waypoints.

Logically the line between the waypoint A --> B equal the line B --> A.
But in fact due to the Bresemham algorithm you can ran into few problems. Because the algorithm can detect a wall between A --> B but not between B --> A.

To solve it we though about few possiblities :
 -  You can simulate a circle around the "activated" pixel to know if there is a wall.
 -  You can also make your line larger so it will detect the wall. 

### Dijkstra algorithm

To find the shortest path between the starting and ending point we decided to implement Dijkstra algorithm.
You can find many informations on this algorithm online :

 - https://www-m9.ma.tum.de/graph-algorithms/spp-dijkstra/index_en.html

### Sources
 - https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm



