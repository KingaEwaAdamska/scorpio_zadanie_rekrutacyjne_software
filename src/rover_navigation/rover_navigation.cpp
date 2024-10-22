#include "rover_navigation.hpp"

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/GetMap.h"
#include<vector>
#include <queue>
#include <sstream>



void RoverNavigation::spin() {
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//odbiera mapę
void RoverNavigation::GetMapServiceClient() {
    autonomy_simulator::GetMap map_data;

    if (_mapInput.call(map_data))
    {
        ROS_INFO("Received map data");


        for (int x = 0; x < GRID_SIZE; x++) {
            for (int y = 0; y < GRID_SIZE; y++) {
                _map[x][y] = map_data.response.data[y * GRID_SIZE + x];
            }
        }

        // ROS_INFO("Map data:");
        // for (int y = GRID_SIZE-1 ; y >= 0; y--) {
        //     std::stringstream row;
        //     for (int x = 0; x < GRID_SIZE; x++) {
        //         row << static_cast<int>(_map[x][y]) << " "; 
        //     }
        //     ROS_INFO("%s", row.str().c_str());
        // }   
    }
    else
    {
        ROS_ERROR("Failed to call service /get_map");
    }
}

//przyjmuje dane o celu łazika
void RoverNavigation::roverGoalCallback(const autonomy_simulator::SetGoal::ConstPtr& msg) {
    ROS_INFO("Received goal: (%d, %d)", msg->x, msg->y);
    if(msg->x == 0 && msg->y == 0)
    {
        ROS_WARN("Warning: Received goal with both x and y set to 0");
    }

    _goalX = msg->x;
    _goalY = msg->y;

    navigateToGoal();
}

//na podstawie mapy i celu zwraca najkrótszą trasę
void RoverNavigation::searchForShortestPath() {

    struct Cell {
        int x, y, dist;
        bool operator>(const Cell &other) const {
            return dist > other.dist;
        }
    };

    //Change of coordinates
    // [0] - up
    // [1] - right
    // [2] - down
    // [3] - left
    int directionChangeX[] = {0, 1, 0, -1};
    int directionChangeY[] = {1, 0, -1, 0};

    int distanceFromStart[GRID_SIZE][GRID_SIZE];

    // Inicjalizacja tablicy wartością INT_MAX
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            distanceFromStart[i][j] = INT_MAX;
        }
    }

    std::vector<std::vector<std::pair<int, int>>> previousCell(GRID_SIZE, std::vector<std::pair<int, int>>(GRID_SIZE, {-1, -1}));

    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> cellsToCheckQueue;

    distanceFromStart[0][0] = 0;
    cellsToCheckQueue.push({0,0,0});

    while(!cellsToCheckQueue.empty()) {

        Cell currentCell = cellsToCheckQueue.top();
        cellsToCheckQueue.pop();

        int currentX = currentCell.x;
        int currentY = currentCell.y;
        int currentDist = currentCell.dist;

        for (int moveDirection = 0; moveDirection < 4; moveDirection++) {
            int newX = currentX + directionChangeX[moveDirection];
            int newY = currentY + directionChangeY[moveDirection];

            if (canMoveTo(newX, newY, _map[currentX][currentY])) {
                int newDist = currentDist + 1;

                if (newDist < distanceFromStart[newX][newY]) {
                    distanceFromStart[newX][newY] = newDist;
                    previousCell[newX][newY] = {currentX, currentY};
                    cellsToCheckQueue.push({newX, newY, newDist});
                }
            }
        }
    }

    if (distanceFromStart[_goalX][_goalY] == INT_MAX) {
        return;
    }

    int x = _goalX;
    int y = _goalY;
     
    while (x != -1 || y != -1) {
        pathToGoal.push_back({x,y});
        std::tie(x,y) = previousCell[x][y];
    }
    std::reverse(pathToGoal.begin(), pathToGoal.end());
}

//sprawdza czy łazik jest w stanie przejść między podanymi komórkami
bool RoverNavigation::canMoveTo(int newX, int newY, int currentHeight) {
    if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE) {
        int newHeight = _map[newX][newY];
        if (newHeight-currentHeight <= 10 && newHeight-currentHeight >=-10)
            return true;
    }

    return false;
}

//prowadzi łazika po wyznaczonej ścieżce
void RoverNavigation::navigateToGoal() {
    
    ros::Rate loop_rate(10);


    loop_rate.sleep(); 
    GetMapServiceClient();

    loop_rate.sleep();
    searchForShortestPath();

    if (pathToGoal.empty()) {
        ROS_INFO("Nie istnieje ścieżka do celu");
        return;
    }

    std::stringstream ss;
    ss << "Ścieżka do celu: ";
    
    for (const auto& point : pathToGoal) {
        ss << "(" << point.first << ", " << point.second << ") ";
    }

    // // Wyświetlenie informacji w ROS_INFO
    // ROS_INFO("%s", ss.str().c_str());

    for (const auto& point: pathToGoal) {
        // ROS_INFO("( %d, %d )", point.first, point.second);

        // ROS_INFO("%d", _map[point.first][point.second]);
        changeCell(point.first, point.second);
    }
    ROS_INFO("Goal reached");
}


void RoverNavigation::changeOrientationOrMove(int targetOrientation, uint8_t moveCommand) {
    std_msgs::UInt8 msg;
    msg.data = moveCommand;
    navigationMovePub(_roverMovePublisher, msg.data);
    _roverOrientation = targetOrientation;  // Ustaw nową orientację
}

void RoverNavigation::changeCell(int nextX, int nextY) {
    ros::Rate loop_rate(10);

    while (_roverPoseX < nextX) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(1, 1);
                break;
            case 1:
                changeOrientationOrMove(1, 2);
                _roverPoseX = nextX;
                break;
            case 2:
                changeOrientationOrMove(1, 0);
                break;
            case 3:
                changeOrientationOrMove(3, 3);
                _roverPoseX = nextX;
                break;
        }
        loop_rate.sleep();
    }

    while (_roverPoseX > nextX) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(3, 0);
                break;
            case 1:
                changeOrientationOrMove(1, 3);
                _roverPoseX = nextX;
                break;
            case 2:
                changeOrientationOrMove(3, 1);
                break;
            case 3:
                changeOrientationOrMove(2, 2);
                _roverPoseX = nextX;
                break;
        }
        loop_rate.sleep();
    }

    // Ruch wzdłuż osi Y
    while (_roverPoseY < nextY) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(0, 2);
                _roverPoseY = nextY;
                break;
            case 1:
                changeOrientationOrMove(0, 0);
                break;
            case 2:
                changeOrientationOrMove(3, 3);
                _roverPoseY = nextY;
                break;
            case 3:
                changeOrientationOrMove(0, 1);
                break;
        }
        loop_rate.sleep();
    }

    while (_roverPoseY > nextY) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(3, 3);
                _roverPoseY = nextY;
                break;
            case 1:
                changeOrientationOrMove(2, 1);
                break;
            case 2:
                changeOrientationOrMove(2, 2);
                _roverPoseY = nextY;
                break;
            case 3:
                changeOrientationOrMove(2, 0);
                break;
        }
        loop_rate.sleep();
    }
}


//publikuje ruch łazika
void RoverNavigation::navigationMovePub(ros::Publisher& pub, uint8_t move_data){

    std_msgs::UInt8 msg;
    msg.data = move_data;

    pub.publish(msg);
}

RoverNavigation::RoverNavigation():
    _nh(ros::NodeHandle("rover_navigation")),
    _roverPoseX(0),
    _roverPoseY(0),
    _roverOrientation(0),
    _goalX(0),
    _goalY(0),
    _roverMovePublisher(_nh.advertise<std_msgs::UInt8>("/rover/move", 1000)),
    _roverGoalSubscriber(_nh.subscribe("/set_goal", 1000, &RoverNavigation::roverGoalCallback, this)),
    _mapInput(_nh.serviceClient<autonomy_simulator::GetMap>("/get_map"))
{
}

 