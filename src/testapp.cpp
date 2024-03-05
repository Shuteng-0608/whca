/*
 * testapp.cpp
 *
 * Purpose:
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

// #include "dummy.h"
// #include "app.h"
#include "ros/ros.h"
#include "app_ros.h"
// #include "graph/simplegrid_ros.h"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "mapf_test_agent");

    run();
    
    return 0;
}