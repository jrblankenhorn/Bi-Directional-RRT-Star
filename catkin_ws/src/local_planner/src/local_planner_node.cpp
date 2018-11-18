#include <ros/ros.h>
#include <local_planner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    RRT::LocalPlanner planner(nh, pnh);

    while(ros::ok)
    {
        ros::spin();
    }

    return 0;

}
