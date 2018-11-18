#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <stdlib.h>
#include <time.h>
#include <tf/transform_listener.h>

typedef std::pair<int, int> point;

namespace RRT {

class LocalPlanner{

public:
    LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~LocalPlanner();

private:

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void goalCallback(const geometry_msgs::Point::ConstPtr &msg);

    void planPath();
    std::map<point, point> expandTree(std::map<point, point> tree, const int &count);
    std::pair<point, point> getNewPoint(std::map<point, point> tree, const int &count);
    void clearVisited();
    void clearTree();
    point getRandomPoint();
    point calcNearestNode(const point &pt, std::map<point, point> tree);
    std::vector<point> calcNeighborsInRadius(const point &new_pt, const point &nearest_node);
    point calcCheapestNeighbor(const std::vector<point> &points);
    double calcCost(const point &pt);
    point interpolatePoint(const point &rand_pt, const point &nearest_node);
    std::vector<point> calcPointsBetween(const point &pt1, const point &pt2);
    bool checkNewPointsForCollision(const std::vector<point> &points_between);
    std::tuple<bool, point, point, point> checkNewPointsForConnection(const std::vector<point> &points_between);
    std::pair<bool, point> checkForConnection();
    void markVisitedPoints(const std::vector<point> &points);    
    void markVisited(const point &pt);
    void setupCostmap(ros::NodeHandle &pnh);
    void setupCSpace();
    void getTreeParams(ros::NodeHandle &pnh);
    void mapCostmapToCSpace(const nav_msgs::OccupancyGrid::ConstPtr &local_costmap);
    int calcGridLocation(const double &x, const double &y);
    point calcCartesianCoords(const point &pt);
    bool checkCollision(const point &pt);

    void calcOccGrid();
    void publishOccGrid(const nav_msgs::OccupancyGrid &grid);
    void calcPathMsg(const point &connection_pt);
    void publishPath(const nav_msgs::Path &path);

    ros::Subscriber costmap_sub;
    ros::Subscriber goal_sub;
    ros::Publisher occ_grid_pub;
    ros::Publisher path_pub;

    geometry_msgs::Point m_goal_pt;

    std::vector<std::vector<int>> m_c_space = {};
    std::vector<std::vector<int>> m_c_space_visited = {};
    std::map<point, point> m_tree_forward;
    std::map<point, point> m_tree_backward;

    double m_local_costmap_res;
    int m_local_costmap_height;
    int m_local_costmap_width;
    double m_max_branch_length;
    double m_search_radius;

    int m_tree_counter = 0;

};

}
