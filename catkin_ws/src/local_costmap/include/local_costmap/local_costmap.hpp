#pragma once

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

typedef std::pair<double, double> point;

namespace RRT {

class LocalCostmap{

public:

    LocalCostmap(ros::NodeHandle &nh, ros::NodeHandle pnh);
    ~LocalCostmap();

    void pubOccGrid();

private:

    void setupCostmap();
    void createObstacles();
    std::vector<point> createRectangle(const double &height, const double &width, const point &center);
    std::vector<point> calcPointsBetween(const point &pt1, const point &pt2);
    void calcOccGrid();
    int calcGridLocation(const double &x, const double &y);
    point calcCartesianCoords(const int &location);
    void clearMap();    

    ros::Publisher occ_grid_pub;

    tf::TransformListener listener;
    nav_msgs::OccupancyGrid occ_grid;

    double m_local_costmap_res;
    double m_local_costmap_height;
    double m_local_costmap_width;
    int m_grid_array_length;

};

}
