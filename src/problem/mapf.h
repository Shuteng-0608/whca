#pragma once
#include "problem.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class MAPF : public Problem {
private:
  void init();

public:
  MAPF(Graph* _G,
       Agents _A,
       std::vector<Task*> _T);
  MAPF(Graph* _G,
       Agents _A,
       std::vector<Task*> _T,
       std::mt19937* _MT);
  ~MAPF();
  bool isSolved();
  void update();
  bool allocated() { return true; }

  std::string logStr();
  void publishGoals(int makespan, int width);
};
