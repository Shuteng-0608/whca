#pragma once
#include "grid.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class CostmapGrid : public Grid {

public:
    CostmapGrid();

private:
    void sub_callback(const nav_msgs::OccupancyGrid &map);
    ros::Subscriber sub;
};
