#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "util/param.h"
#include "util/util.h"
#include "graph/simplegrid_ros.h"
#include "agent/agent.h"
#include "task/task.h"
#include "problem/mapf.h"
#include "solver/whca.h"
#include "ros/ros.h"

Problem* run() {
    ROS_INFO("====== Reading config file...... ======");
    std::string configfile = "/home/shuteng/catkin_ws/src/pibt_ros/sample-param.txt";

    /************************
     * default setting
     ************************/
    Param::EnvConfig* envConfig = new Param::EnvConfig
        {
        Param::P_MAPF,    // problem type
        Param::S_WHCA,    // solver type
        "/home/shuteng/catkin_ws/src/pibt_ros/map/5x5.map",  // field type
        2,                // agent number
        10000,            // timesteplimit
        10,               // task number
        0.1,              // task frequency
        false,            // scenario
        "",               // scenario file
        0,                // seed
        false,            // save log
        true,             // print log
        false,            // print time
        };
    
    Param::SolverConfig* solverConfig = new Param::SolverConfig
        {
        false,  // WarshallFloyd
        true,   // CBS, ECBS, iECBS, independet operater
        5,      // WHCA* or winPIBT, window size
        1.5,    // ECBS or iECBS, suboptimal param
        true,   // winPIBT, softmode
        };
    
    Param::VisualConfig* visualConfig = new Param::VisualConfig
        {
        false,                  // showicon
        "./material/sheep.png"  // icon
        };

    ROS_INFO("====== Loading config setting...... ======");
    setParams(configfile, envConfig, solverConfig, visualConfig);

    /************************
     * graph definition
     ************************/
    ROS_INFO("====== Reading costmap...... ======");
    Graph* G = new CostmapGrid();
    
    ros::Duration duration(1);
    duration.sleep();
    ros::spinOnce();
    

    /************************
     * agent definition
     ************************/
    Agents A;
    Paths points;
    ROS_INFO("====== Get Fixed Starts & Goals ======");
    Node* agent1 = new Node(1);
    Node* agent2 = new Node(2);
    Node* goal1 = new Node(3);
    Node* goal2 = new Node(4);
    Vec2f pos1(0, 0);
    Vec2f pos2(0, 1);
    Vec2f pos3(0, 2);
    Vec2f pos4(2, 2);
    agent1->setPos(pos1);
    agent2->setPos(pos2);
    goal1->setPos(pos3);
    goal2->setPos(pos4);
    Nodes fixedStarts = {agent1, agent2};
    Nodes fixedGoals = {agent1, agent2};
    points = G->getFixedStartGoal(fixedStarts, fixedGoals);
    for (int i = 0; i < envConfig->agentnum; ++i) {
        Agent* a = new Agent(points[i][0]);
        A.push_back(a);
    }
    ROS_INFO("Agent 1 : [0, 0] ---> [0, 4]");
    ROS_INFO("Agent 2 : [0, 4] ---> [2, 2]");

    /************************
     * problem definition
     ************************/
    ROS_INFO("====== Problem definition ======");
    std::vector<Task*> T;
    for (int i = 0; i < envConfig->agentnum; ++i) {
        Task* tau = new Task(points[i][1]);
        T.push_back(tau);
    }
    MAPF* P = new MAPF(G, A, T);
    
    ROS_INFO("Problem definition : MAPF");

    // set timestep limit
    P->setTimestepLimit(envConfig->timesteplimit);

    /************************
     * solver definition
     ************************/
    ROS_INFO("====== Solver definition ======");
    Solver* solver = new WHCA(P, solverConfig->window);
    ROS_INFO("Problem solver : WHCA");

    /************************
     * start solving
     ************************/
    ROS_INFO("====== Start solving ======");
    solver->solve();
    ROS_INFO("====== Solve Finished ======");

    

    // Publish path to each agent
    ROS_INFO("====== Publish path to each agent ======");
    int makespan = P->getTerminationTime();
    int width = G->getW();
    P->publishGoals(makespan, width);


    return P;
}