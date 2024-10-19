#include"rover_navigation.hpp"

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "autonomy_simulator/SetGoal.h"

int goal_x = 0;
int goal_y = 0;
int direction = 0; //w górę 0, w prawo 1 itd.


void navigation_move(ros::Publisher&, uint8_t);
void roverGoalSubscriber(const autonomy_simulator::SetGoal::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rover_navigation");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/set_goal", 1000, roverGoalSubscriber);

    ros::Publisher move_pub = n.advertise<std_msgs::UInt8>("/rover/move", 1000);

    std_msgs::UInt8 msg;
    

    int x = 0;
    int y = 0;

    ros::Rate loop_rate(10);

    
    while (ros::ok()){
        if(goal_x != 0 || goal_y != 0){
        
            if (y != goal_y){
                msg.data = 2;
                navigation_move(move_pub, msg.data);
                y++;
                ROS_INFO("%d %d", x, y);
                loop_rate.sleep(); 
            }else if (x != goal_x){
                if (direction != 1){
                    direction = 1;
                    msg.data = 1;
                    navigation_move(move_pub, msg.data); 
                }else{
                    msg.data = 2;
                    navigation_move(move_pub, msg.data);
                    x++;
                ROS_INFO("%d %d", x, y);
                }
            }
            
            
        } 
        ros::spinOnce();
        loop_rate.sleep();  
    }

    ROS_INFO("Na miejscu");

    return 0;
}

void navigation_move(ros::Publisher& pub, uint8_t move_data){

    std_msgs::UInt8 msg;
    msg.data = move_data;

    pub.publish(msg);
}

void roverGoalSubscriber(const autonomy_simulator::SetGoal::ConstPtr& msg)
{
    ROS_INFO("Received goal: (%d, %d)", msg->x, msg->y);
    goal_x = msg->x;
    goal_y = msg->y;
}
