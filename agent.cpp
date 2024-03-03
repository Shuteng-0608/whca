/*
 * agent.cpp
 *
 * Purpose: Agent
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "agent.h"
#include "../util/util.h"
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"

// #include "../graph/grid.h"

int Agent::cntId = 0;

Agent::Agent() : id(cntId) {
  ++cntId;
  g = nullptr;
  tau = nullptr;
  updated = false;
  beforeNode = nullptr;
}

Agent::Agent(Node* _v) : id(cntId) {
  ++cntId;
  g = nullptr;
  tau = nullptr;
  v = nullptr;
  setNode(_v);
  updated = false;
}

Agent::~Agent() {
  for (auto s : hist) delete s;
  hist.clear();
}



void Agent::setNode(Node* _v) {
  // error check
  if (v != nullptr) {
    auto neigh = v->getNeighbor();
    if (!(_v == v || inArray(_v, neigh))) {
      std::cout << "error@Agent, set invalid node, from "
                << v->getId() << " to " << _v->getId() << std::endl;
      std::exit(1);
    }
  }

  beforeNode = v;
  v = _v;
}

void Agent::updateHist() {
  AgentStatus* s = new AgentStatus;
  s->v = v;
  if (hasGoal()) {
    s->g = g;
  } else {
    s->g = nullptr;
  }
  if (hasTask()) {
    s->tau = tau;
  } else {
    s->tau = nullptr;
  }
  hist.push_back(s);
}

void Agent::releaseTask() {
  g = nullptr;
  tau = nullptr;
}

void Agent::releaseTaskOnly() {
  tau = nullptr;
}

void Agent::releaseGoalOnly() {
  g = nullptr;
}



// //ros
// int width = 8;
// void width_callback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
//     width = map->info.width;
//     ROS_INFO("Received map width: %d", width);
// }



    
std::vector<std::vector<std::pair<int, int>>> Agent::getCoordinates() {
        return coordinates;
    }

std::pair<int, int> convertTo2D(int index, int width) {
    int x = index % width;  // 横坐标
    int y = index / width;  // 纵坐标
    return std::make_pair(x, y);
}

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// std::string Agent::logStr() {
//   ros::NodeHandle nh;
//   ros::Subscriber map_width_sub = nh.subscribe("/map", 10, width_callback);
//   ros::Duration d(1);
//   d.sleep();
//   ros::spinOnce();


//   std::string str, strPath, strGoal;
//   str += "[agent] ";
//   str += "id:" + std::to_string(id) + "\n";
//   strPath = "path:";
//   strGoal = "goal:";

//   std::vector<std::pair<int, int>> pathCoords;
//   std::vector<std::pair<int, int>> goalCoords;
//   for (auto s : hist) {
//       std::pair<int, int> coordinates = convertTo2D(s->v->getId(), width);
//       pathCoords.push_back(coordinates);
//       strPath += "(" + std::to_string(coordinates.first) + "," + std::to_string(coordinates.second) + "),";
//       if (s->g != nullptr) {
//           std::pair<int, int> goalCoordinates = convertTo2D(s->g->getId(), width);
//           goalCoords.push_back(goalCoordinates);
//           strGoal += "(" + std::to_string(goalCoordinates.first) + "," + std::to_string(goalCoordinates.second) + "),";
//       } else {
//           strGoal += "*,";
//       }
//     }
    
//   //  // 将存储的坐标数组设置给Agent
//   // // std::vector<std::vector<std::pair<int, int>>> coords;
//   // std::pair<int, std::vector<std::pair<int, int>>> composition;
//   // // std::pair<int, std::vector<std::pair<int, int>>> pre_composition;
//   // composition.first = id;
//   // composition.second = pathCoords;
//   // // coords.push_back(pathCoords);
//   // // coords.push_back(goalCoords);
//   // // 打印 composition 的所有信息
//   // std::cout << "output" << std::endl;
//   // std::cout << "(" << composition.first << ", ";
//   // for (const auto& pair : composition.second) {
//   //     std::cout << "(" << pair.first << ", " << pair.second << ") ";
//   // }
//   // std::cout << ")" << std::endl;

 

//   str += strPath + "\n";
//   str += strGoal + "\n";
//   return str;
// }
std::string Agent::logStr() {
  std::string str, strPath, strGoal;
  str += "[agent] ";
  str += "id:" + std::to_string(id) + "\n";
  strPath = "path:";
  strGoal = "goal:";
  for (auto s : hist) {
    strPath += std::to_string(s->v->getId()) + ",";
    if (s->g != nullptr) {
      strGoal += std::to_string(s->g->getId()) + ",";
    } else {
      strGoal += "*,";
    }
  }
  str += strPath + "\n";
  str += strGoal + "\n";
  return str;
}

std::pair<int, std::vector<std::pair<int, int>>> Agent::getComposition(int width){
  std::vector<std::pair<int, int>> pathCoords;
  std::vector<std::pair<int, int>> goalCoords;

  // ros::NodeHandle nh;
  // ros::Subscriber map_width_sub = nh.subscribe("/map", 10, width_callback);
  // ros::Duration d(1);
  // d.sleep();
  // ros::spinOnce();


  for (auto s : hist) {
      std::pair<int, int> coordinates = convertTo2D(s->v->getId(), width);
      pathCoords.push_back(coordinates);
      
      if (s->g != nullptr) {
          std::pair<int, int> goalCoordinates = convertTo2D(s->g->getId(), width);
          goalCoords.push_back(goalCoordinates);
          
      } 
  }
  // std::vector<std::pair<int, int>> pathCoords;
  std::pair<int, std::vector<std::pair<int, int>>> composition;
  // std::pair<int, std::vector<std::pair<int, int>>> pre_composition;
  composition.first = id;
  composition.second = pathCoords;

  return composition;



}


