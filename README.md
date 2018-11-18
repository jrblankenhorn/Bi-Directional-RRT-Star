# Bi-Directional-RRT-Star

compile with catkin_make
run with roslaunch local_planner bi_directional_rrt_star.launch

current bounds of planner/costmap: 200x200m (-100 to 100 in x and y)

publish a geometry_msgs::Point to /local_planner/goal (within bounds of costmap), and planner will generate a path
