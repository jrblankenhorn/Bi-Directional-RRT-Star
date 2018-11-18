#include <ros/ros.h>
#include <local_costmap.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_costmap_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    RRT::LocalCostmap cmap(nh, pnh);

    ros::Rate r(10);

    while(ros::ok)
    {
        ros::spinOnce();

        cmap.pubOccGrid();

        r.sleep();
    }

    return 0;

}
