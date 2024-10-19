#include "rover_navigation.hpp"

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "autonomy_simulator/SetGoal.h"


void RoverNavigation::spin()
{
    roverRouteAnalyzer();  // Dodaj tutaj wywołanie
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void RoverNavigation::navigationMovePub(ros::Publisher& pub, uint8_t move_data){

    std_msgs::UInt8 msg;
    msg.data = move_data;

    pub.publish(msg);
}

void RoverNavigation::roverGoalCallback(const autonomy_simulator::SetGoal::ConstPtr& msg)
{
    ROS_INFO("Received goal: (%d, %d)", msg->x, msg->y);  // Wyświetlanie odebranych danych

    // Dodatkowy test, czy dane faktycznie są przesyłane
    if(msg->x == 0 && msg->y == 0)
    {
        ROS_WARN("Warning: Received goal with both x and y set to 0");
    }

    _goalX = msg->x;
    _goalY = msg->y;

    ROS_INFO("jestem");  // Testowy komunikat, że callback się wykonał
}


void RoverNavigation::roverRouteAnalyzer(){

    std_msgs::UInt8 msg;
    
    ros::Rate loop_rate(10);

    while (ros::ok()){
        if(_goalX != 0 || _goalY != 0){
        
            if (_roverPoseY != _goalY){
                msg.data = 2;
                navigationMovePub(_roverMovePublisher, msg.data);
                _roverPoseY++;
                ROS_INFO("%d %d", _roverPoseX, _roverPoseY);
                loop_rate.sleep(); 
            }else if (_roverPoseX != _goalX){
                if (_roverDirection != 1){
                    _roverDirection = 1;
                    msg.data = 1;
                    navigationMovePub(_roverMovePublisher, msg.data); 
                }else{
                    msg.data = 2;
                    navigationMovePub(_roverMovePublisher, msg.data);
                    _roverPoseX++;
                    ROS_INFO("%d %d", _roverPoseX, _roverPoseY);
                }
            }
            
            
        } 
        ros::spinOnce();
        loop_rate.sleep();  
    }

    ROS_INFO("Na miejscu");
}

RoverNavigation::RoverNavigation():
    _nh(ros::NodeHandle("rover_navigation")),
    _roverPoseX(0),
    _roverPoseY(0),
    _roverDirection(0),
    _goalX(0),
    _goalY(0),
    _roverMovePublisher(_nh.advertise<std_msgs::UInt8>("/rover/move", 1000)),
    _roverGoalSubscriber(_nh.subscribe("/set_goal", 1000, &RoverNavigation::roverGoalCallback, this))
{
    ROS_INFO("Subscriber initialized for /setgoal topic");
}

