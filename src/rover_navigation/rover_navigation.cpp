#include "rover_navigation.hpp"

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8MultiArray.h>
#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverMap.h"
#include<vector>
#include <queue>
#include <sstream>
#include<memory>



void RoverNavigation::spin() {
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
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

void RoverNavigation::sensorDataSaver() {

    struct Cell {
        int x,y;
    };

    //sensorCellChangelica komórek podanych przez sensor zależnie od orientacji
    Cell sensorCellChange[4][6]={ 
                    {{-1,1}, {0,1}, {1,1}, {-1,2}, {0,2}, {1,2}},
                    {{1,1}, {1,0}, {1,-1}, {2,1}, {2,0}, {2,-1}},
                    {{1,-1}, {0,-1}, {-1,-1}, {1,-2}, {0,-2}, {-1,-2}},
                    {{-1,-1}, {-1,0}, {-1,1}, {-2,-1}, {-2,0}, {-2,1}}
                };

    boost::shared_ptr<const autonomy_simulator::RoverMap_<std::allocator<void> > > msg = ros::topic::waitForMessage<autonomy_simulator::RoverMap>("/rover/sensor");

    for (int i = 0; i < 6 ;++i){
        if (_roverPoseX + sensorCellChange[_roverOrientation][i].x < GRID_SIZE && 
            _roverPoseX + sensorCellChange[_roverOrientation][i].x >= 0 &&
            _roverPoseY + sensorCellChange[_roverOrientation][i].y < GRID_SIZE &&
            _roverPoseY + sensorCellChange[_roverOrientation][i].y >=0)
        _map[_roverPoseX + sensorCellChange[_roverOrientation][i].x][_roverPoseY + sensorCellChange[_roverOrientation][i].y] = _roverCurrentHeight + msg->data[i];
    }

    //Wyświetla aktualnie zeskanowaną mapę

    // ROS_INFO("Map scaned data:");
    // for (int y = GRID_SIZE-1 ; y >= 0; y--) {
    //     std::stringstream row;
    //     for (int x = 0; x < GRID_SIZE; x++) {
    //         row << static_cast<int>(_map[x][y]) << " "; 
    //     }
    //     ROS_INFO("%s", row.str().c_str());
    // }   
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

    // Inicjalizacja sensorCellChangelicy wartością INT_MAX
    for (int x = 0; x < GRID_SIZE; ++x) {
        for (int y = 0; y < GRID_SIZE; ++y) {
            distanceFromStart[x][y] = INT_MAX;
        }
    }

    std::vector<std::vector<std::pair<int, int>>> previousCell(GRID_SIZE, std::vector<std::pair<int, int>>(GRID_SIZE, {-1, -1}));

    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> cellsToCheckQueue;

    distanceFromStart[_roverPoseX][_roverPoseY] = 0;
    cellsToCheckQueue.push({_roverPoseX,_roverPoseY,0});

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

    // std::stringstream ss;
    // ss << "Ścieżka do celu: ";
    
    // for (const auto& point : pathToGoal) {
    //     ss << "(" << point.first << ", " << point.second << ") ";
    // }

    // // // Wyświetlenie informacji w ROS_INFO
    //  ROS_INFO("%s", ss.str().c_str());
}

//sprawdza czy łazik jest w stanie przejść między podanymi komórkami
bool RoverNavigation::canMoveTo(int newX, int newY, int currentHeight) {
    if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE) {
        int newHeight = _map[newX][newY];
        if ((newHeight-currentHeight <= 10 && newHeight-currentHeight >=-10) || newHeight == -1)
            return true;
    }

    return false;
}

//prowadzi łazika po wyznaczonej ścieżce
void RoverNavigation::navigateToGoal() {
    
    ros::Rate loop_rate(10);

    //pusta mapa
    for (int x = 0; x < 50; ++x) {
        for (int y = 0; y < 50; ++y) {
            _map[x][y] = -1;
        }
    }
    _map[0][0] = 0;

    sensorDataSaver();

    loop_rate.sleep();

    searchForShortestPath();

    if (pathToGoal.empty()) {
        ROS_INFO("Path to goal does not exist");
        return;
    }

    while (_roverPoseX != _goalX || _roverPoseY != _goalY) {
        changeCell(pathToGoal[1].first, pathToGoal[1].second);
        pathToGoal.clear();
        searchForShortestPath();

    //Wyświetlenie wyznaczonej ścieżki
    // std::stringstream ss;
    // ss << "Ścieżka do celu: ";
    
    // for (const auto& point : pathToGoal) {
    //     ss << "(" << point.first << ", " << point.second << ") ";
    //}
    // ROS_INFO("%s", ss.str().c_str());

        if (pathToGoal.empty()) {
            ROS_INFO("Path to goal does not exist");
            return;
        }
    }
    ROS_INFO("Goal reached");
}

//zmienia orientację lub wysyła informacje o ruchu łazika
void RoverNavigation::changeOrientationOrMove(int targetOrientation, uint8_t moveCommand) {
    std_msgs::UInt8 msg;
    msg.data = moveCommand;
    navigationMovePub(_roverMovePublisher, msg.data);
    _roverOrientation = targetOrientation;  // Ustaw nową orientację
}

void RoverNavigation::changeCell(int nextX, int nextY) {

    ros::Rate loop_rate(10);

    if (_roverPoseX < nextX) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(1, 1);
                break;
            case 1:
                changeOrientationOrMove(1, 2);
                _roverCurrentHeight = _map[nextX][nextY];
                _roverPoseX = nextX;
                break;
            case 2:
                changeOrientationOrMove(1, 0);
                break;
            case 3:
                changeOrientationOrMove(0, 1);
                break;
        }
    }

    if (_roverPoseX > nextX) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(3, 0);
                break;
            case 1:
                changeOrientationOrMove(0, 0);
                break;
            case 2:
                changeOrientationOrMove(3, 1);
                break;
            case 3:
                changeOrientationOrMove(2, 2);
                _roverCurrentHeight = _map[nextX][nextY];
                _roverPoseX = nextX;
                break;
        }
        
    }

    // Ruch wzdłuż osi Y
    if (_roverPoseY < nextY) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(0, 2);
                _roverCurrentHeight = _map[nextX][nextY];
                _roverPoseY = nextY;
                break;
            case 1:
                changeOrientationOrMove(0, 0);
                break;
            case 2:
                changeOrientationOrMove(3, 1);
                break;
            case 3:
                changeOrientationOrMove(0, 1);
                break;
        }
    }

    if (_roverPoseY > nextY) {
        switch (_roverOrientation) {
            case 0:
                changeOrientationOrMove(3, 0);
                break;
            case 1:
                changeOrientationOrMove(2, 1);
                break;
            case 2:
                changeOrientationOrMove(2, 2);
                _roverCurrentHeight = _map[nextX][nextY];
                _roverPoseY = nextY;
                break;
            case 3:
                changeOrientationOrMove(2, 0);
                break;
        }
    }
    loop_rate.sleep();
    sensorDataSaver();
    roverMapPub(_roverMapPublisher);
}

//publikuje ruch łazika
void RoverNavigation::navigationMovePub(ros::Publisher& pub, uint8_t move_data){

    std_msgs::UInt8 msg;
    msg.data = move_data;

    pub.publish(msg);
}

//publikuje mapę
void RoverNavigation::roverMapPub(ros::Publisher& pub){

    autonomy_simulator::RoverMap msg;
    msg.data.resize(GRID_SIZE * GRID_SIZE);
    for (int x = 0; x < GRID_SIZE; x++) {
            for (int y = 0; y < GRID_SIZE; y++) {
                msg.data[y * GRID_SIZE + x] = _map[x][y];
            }
        }

    pub.publish(msg);
}

RoverNavigation::RoverNavigation():
    _nh(ros::NodeHandle("rover_navigation")),
    _roverPoseX(0),
    _roverPoseY(0),
    _roverCurrentHeight(0),
    _roverOrientation(0),
    _goalX(0),
    _goalY(0),
    _roverMovePublisher(_nh.advertise<std_msgs::UInt8>("/rover/move", 1000)),
    _roverMapPublisher(_nh.advertise<autonomy_simulator::RoverMap>("/rover/map", 1000)),
    _roverGoalSubscriber(_nh.subscribe("/set_goal", 1000, &RoverNavigation::roverGoalCallback, this))
{
}

 