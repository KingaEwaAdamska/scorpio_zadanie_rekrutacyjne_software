#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "autonomy_simulator/SetGoal.h"
#include "utils.hpp"


class RoverNavigation {
    ros::NodeHandle _nh;
    //Goal Data
    int8_t _goalX;
    int8_t _goalY;

    //Rover Data
    int8_t _roverDirection; 
    int8_t _roverPoseX;
    int8_t _roverPoseY;

    

    // Route choose
    void roverRouteAnalyzer();

    // RoverMove Publisher
    ros::Publisher _roverMovePublisher;
    void navigationMovePub(ros::Publisher&, uint8_t);

    // SetGoal subscriber
    ros::Subscriber _roverGoalSubscriber;
    void roverGoalCallback(const autonomy_simulator::SetGoal::ConstPtr& msg);

 public:
 void spin();
    RoverNavigation();
    void start();
};
