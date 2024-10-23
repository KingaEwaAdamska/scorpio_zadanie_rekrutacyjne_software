#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/GetMap.h"
#include "utils.hpp"
#include<vector>
#define GRID_SIZE 50

class RoverNavigation {
    ros::NodeHandle _nh;

    //Goal Data
    int8_t _goalX;
    int8_t _goalY;

    //Rover Data
    int8_t _roverOrientation; 
    int8_t _roverPoseX;
    int8_t _roverPoseY;
    int8_t _roverCurrentHeight;

    //Map Data
    int8_t _map[GRID_SIZE][GRID_SIZE];
    std::vector<std::pair<int, int>> pathToGoal;


    //Route choose
    bool canMoveTo(int, int, int);
    void searchForShortestPath();

    //Rover drive
    void navigateToGoal();
    void changeCell(int,int);
    void changeOrientationOrMove(int targetOrientation, uint8_t moveCommand);

    //RoverMove Publisher
    ros::Publisher _roverMovePublisher;
    void navigationMovePub(ros::Publisher&, uint8_t);

    //RoverMap Publisher
    ros::Publisher _roverMapPublisher;
    void roverMapPub(ros::Publisher&);
    

    //SetGoal subscriber
    ros::Subscriber _roverGoalSubscriber;
    void roverGoalCallback(const autonomy_simulator::SetGoal::ConstPtr& msg);

    //Rover sensor
    void sensorDataSaver();


 public:
    void spin();
    RoverNavigation();
    void start();
};
