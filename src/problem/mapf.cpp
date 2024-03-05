/*
 * mapf.cpp
 *
 * Purpose: MAPF
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "mapf.h"
#include "../util/util.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


MAPF::MAPF(Graph* _G,
           Agents _A,
           std::vector<Task*> _T) : Problem(_G, _A, _T)
{
  init();
}

MAPF::MAPF(Graph* _G,
           Agents _A,
           std::vector<Task*> _T,
           std::mt19937* _MT) : Problem(_G, _A, _T, _MT)
{
  init();
}

void MAPF::init() {
  // error check
  if (A.size() != T_OPEN.size()) {
    std::cout << "error@MAPF::init, this is not a MAPF, |A| != |T|" << "\n";
    std::exit(1);
  }

  // allocation, t=0
  for (int i = 0; i < A.size(); ++i) {
    A[i]->setTask(T_OPEN[i]);
    A[i]->setGoal(A[i]->getTask()->getG()[0]);
    A[i]->updateHist();
  }
}

MAPF::~MAPF() {}

bool MAPF::isSolved() {
  if (!T_OPEN.empty()) return false;
  return std::all_of(A.begin(), A.end(), [] (Agent* a) { return a->getGoal() == a->getNode(); });
}

void MAPF::update() {
  ++timestep;

  for (auto a : A) {
    auto tau = a->getTask();
    if (tau) {  // if agent has a task
      tau->update(a->getNode());  // update task status
      if (tau->completed()) {  // if task is completed
        a->releaseTaskOnly();  // agent release task, not goal
        openToClose(tau, T_OPEN, T_CLOSE);
      }
    } else if (a->getNode() != a->getGoal()) {
      // create new task and assign
      Task* newTau = new Task(a->getGoal());
      a->setTask(newTau);
      a->setGoal(newTau->getG()[0]);
      T_OPEN.push_back(newTau);
    }
    a->updateHist();
  }
}

std::string MAPF::logStr() {
  std::string str = Problem::logStr();
  str += "[problem] type:MAPF\n";
  str += "[problem] agentnum:" + std::to_string(A.size()) + "\n";
  str += G->logStr();
  for (auto tau : T_CLOSE) str += tau->logStr();

  int cnt;
  int pathsize;
  // 
  for (auto a : A) {
    auto hist = a->getHist();
    auto itr = hist.end() - 1;
    cnt = 0;
    while ((*itr)->v == a->getGoal()) {
      ++cnt;
      --itr;
    }
    pathsize = getTerminationTime() - cnt + 1;
    str += a->logStr();
    str += "size:" + std::to_string(pathsize) + "\n";
  }

  

  return str;
}


void sendGoal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac, int makespan) {
  
  ROS_INFO("================== Sending Goal ================= makespan %d", makespan);
  ac.sendGoal(goal);
  ac.waitForResult();
  ROS_INFO("================== Goal Arrived ================= makespan %d", makespan);
  
}

void MAPF::publishGoals (int makespan, int width){
  int cnt;
  int pathsize;
  std::pair<int, std::vector<std::pair<int, int>>> composition_0;
  std::pair<int, std::vector<std::pair<int, int>>> composition_1;
  for (auto a : A) {
    auto hist = a->getHist();
    auto itr = hist.end() - 1;
    cnt = 0;
    while ((*itr)->v == a->getGoal()) {
      ++cnt;
      --itr;
    }
    pathsize = getTerminationTime() - cnt + 1;
   
    if(a->getComposition(width).first==0){
      ROS_INFO("Get composition 0");
      composition_0 = a->getComposition(width);
    }
    if(a->getComposition(width).first==1){
      ROS_INFO("Get composition 1");
      composition_1 = a->getComposition(width);
    }
  }

  MoveBaseClient ac_0("/rb_0/move_base", true);
  while(!ac_0.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the rb_0/move_base/goal action server to come up");
  }
  MoveBaseClient ac_1("/rb_1/move_base", true);
  while(!ac_1.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the rb_1/move_base/goal action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal_0, goal_1;
  goal_0.target_pose.header.frame_id = "map";
  goal_1.target_pose.header.frame_id = "map";
  for (int i = 0; i < makespan; i++) {
    goal_0.target_pose.header.stamp = ros::Time::now();
    goal_0.target_pose.pose.position.x = static_cast<double>(composition_0.second[0].first);
    goal_0.target_pose.pose.position.y = static_cast<double>(composition_0.second[0].second);
    goal_0.target_pose.pose.orientation.w = 1;

    goal_1.target_pose.header.stamp = ros::Time::now();
    goal_1.target_pose.pose.position.x = static_cast<double>(composition_1.second[0].first);
    goal_1.target_pose.pose.position.y = static_cast<double>(composition_1.second[0].second);
    goal_1.target_pose.pose.orientation.w = 1;

 
    // Create threads to send goals for each robot
    boost::thread thread_0(boost::bind(&sendGoal, goal_0, boost::ref(ac_0), i));
    boost::thread thread_1(boost::bind(&sendGoal, goal_1, boost::ref(ac_1), i));

    // Join threads
    thread_0.join();
    thread_1.join();
  }
}
