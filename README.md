# topological_navigation

#### How it works ?
The goal of this package is to define waypoints is your map via waypoints generator package [https://github.com/epan-utbm/waypoint_generator]

When you are done with that you can move on the topological package which is a service. 
To make it works you have to send to it the starting and ending point and the waypoints file you have created.

After that it will find the shortest path to go from the starting point to the ending via differents waypoints.

For that it needs to find the link betwenn all waypoints. In other words, is there a wall between two waypoints ?

To do this task you can use the algorithm you want by passing it name in the following line(66) :

```c
    private_nh.param<string>("path_planning", path_planning, "Bresenham");
```

For your case we implemented Bresemham algorithm. 


#### Bresemham algorithm 
This algorithm draw an invisible line passing through pixels between two waypoints. If one pixel is occupied (100) or unknown cell (-1), we can seay that there is a wall.

So we simulate all trajectories possibilities for each waypoints.

Logically the line between the waypoint A --> B equal the line B --> A.
But in fact due to the Bresemham algorithm you can ran into few problems. Because if the algorithm detect a wall between A --> B but not between B --> A they will be a non-sense.

To solve it we though about few possiblities :
    - You can simulate a circle around the "activated" pixel to know if there is a wall.
    - You can also make your line larger so it will detect the wall. 





