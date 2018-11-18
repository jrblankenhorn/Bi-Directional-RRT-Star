# Bi-Directional-RRT-Star

holonomic bi directional RRT* implemenation in C++, wrapped in a catkin workspace.

compile with catkin_make
run with roslaunch local_planner bi_directional_rrt_star.launch

current bounds of planner/costmap: 200x200m (-100 to 100 in x and y)

publish a geometry_msgs::Point to /local_planner/goal (within bounds of costmap), and planner will generate a path

rosparams:

local_costmap_res - resolution of costmap/c-space

local_costmap_height - height of costmap/c-space in meters

local_costmap_width - width of costmap/c-space in meters

max_branch_length - max length of a branch in an RRT* tree in meters

search_radius - radius in meters algorithm will search for cheapest cost path/node to new random point

