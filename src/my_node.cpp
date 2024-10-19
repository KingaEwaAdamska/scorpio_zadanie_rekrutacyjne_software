#include "ros/ros.h"
#include "std_msgs/String.h"

class MyRosNode
{
public:
    MyRosNode()
    {
        // Inicjalizacja subskrybenta
        sub_ = nh_.subscribe("chatter", 1000, &MyRosNode::chatterCallback, this);

        // Inicjalizacja publishera
        pub_ = nh_.advertise<std_msgs::String>("response", 1000);
    }

    // Funkcja główna node'a
    void spin()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    // Callback dla subskrybenta
    void chatterCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Received: [%s]", msg->data.c_str());

        // Tworzenie wiadomości do publikacji
        std_msgs::String response_msg;
        response_msg.data = "Hello from MyRosNode";

        // Publikowanie wiadomości
        pub_.publish(response_msg);
    }

    // Deklaracje zmiennych ROS
    ros::NodeHandle nh_;                // Uchwyt do node'a
    ros::Publisher pub_;                // Publisher
    ros::Subscriber sub_;               // Subscriber
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");  // Inicjalizacja node'a

    MyRosNode node;  // Utworzenie obiektu klasy MyRosNode
    node.spin();     // Wywołanie głównej pętli node'a

    return 0;
}
