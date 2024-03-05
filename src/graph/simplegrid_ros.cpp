#include "simplegrid_ros.h"

CostmapGrid::CostmapGrid() {
    ros::NodeHandle nh;
    ROS_INFO("Subscribe to the /map topic.......");
    
    sub = nh.subscribe("/mapf_base/map", 10, &CostmapGrid::sub_callback, this);
}

void CostmapGrid::sub_callback(const nav_msgs::OccupancyGrid &map) {
    ROS_INFO("====== [Sub_callback begin] : Costmap info processing......");

    // Create basic map dim
    int dimx = map.info.width;
    int dimy = map.info.height;
    double resolution = map.info.resolution;
    ROS_INFO("Map Width : %d", dimx);
    ROS_INFO("Map Height : %d", dimy);
    ROS_INFO("Map Resolution : %lf", resolution);

    setSize(dimx, dimy);

    // create nodes
    int obstacles = 0;
    for (int i = 0; i < dimx * dimy; i++) {
        if (map.data[i] > 0) {
            obstacles++;
            continue;
        }
        Node* v = new Node(i);
        v->setPos(i / dimx, i % dimx);
        nodes.push_back(v);
    }
    ROS_INFO("Number of obstacles : %d", obstacles);

    // Set neighbor
    Nodes neighbor;
    std::string s;
    Node* vv;
    int w  = getW();
    for (int i = 0; i < dimx * dimy; i++) {
        if (map.data[i] > 0) continue;
        vv = getNode(i);  // target node
        neighbor.clear();
        if (existNode(i - w)) neighbor.push_back(getNode(i - w));
        if (existNode(i - 1)) neighbor.push_back(getNode(i - 1));
        if (existNode(i + 1)) neighbor.push_back(getNode(i + 1));
        if (existNode(i + w)) neighbor.push_back(getNode(i + w));
        vv->setNeighbor(neighbor);
    }
    ROS_INFO("Setting neighbors ....");

    // Set start and goal
    starts = nodes;
    goals = nodes;

    ROS_INFO("====== [Sub_callback end]: Costmap info processed");

}
