/*
 * testapp.cpp
 *
 * Purpose:
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

// #include "dummy.h"
#include <ros/ros.h>
#ifndef OF

#include "app.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv,"mapf_test_agent");
  run(argc, argv);
  
  

    
    

  return 0;
}

#endif
