#include <ros/ros.h>
#include "rover_navigation.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rover_navigation");
    ROS_INFO("hej");
    RoverNavigation roverNavigation;
    roverNavigation.spin();
    return 0;
}
