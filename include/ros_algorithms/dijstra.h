#pragma once
#include <ros_algorithms/global_planner.h>
#include <cmath>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PointStamped.h>

class NodeDijstra{
protected:
    double g_cost_;
    bool occupied_;
    NodeDijstra* parent_;
    Coordinate coordinate_;
public:
    NodeDijstra(){
        g_cost_ = INT_MAX;
        occupied_ = 0;
        parent_ = nullptr;
        coordinate_ = Coordinate();
    }
    void set_occupied(bool occupied){occupied_ = occupied;}
    void set_gcost(double g_cost){g_cost_ = g_cost;}
    void set_parent(NodeDijstra* parent){parent_ = parent;}
    void set_position(int x, int y){coordinate_.x_ = x; coordinate_.y_ = y;}
    bool get_occupied(){return occupied_;}
    double get_gcost(){return g_cost_;}
    NodeDijstra* get_parent(){return parent_;}
    Coordinate get_coordinate(){return coordinate_;}
};

struct cmp{
    bool operator()(NodeDijstra* n1, NodeDijstra* n2){
        if(n1->get_gcost() >= n2->get_gcost())
            return true;
        else
            return false;
    }
};

class Dijstra : public GlobalPlanner {
protected:
    ros::Publisher start_point_pub;
    ros::Publisher goal_point_pub;

    NodeDijstra** node_map;
    NodeDijstra* current_node;
    std::priority_queue<NodeDijstra*, std::vector<NodeDijstra*>, cmp> open_list;
    // NodeDijstra가 key가 되고, parent NodeDijstra가 value가 되는 closed_list
    std::unordered_map<NodeDijstra*, NodeDijstra*> closed_list;
    
public:
    Dijstra();
    ~Dijstra();
    double gFunction(int x1, int y1, int x2, int y2);
    void virtual publish();
    void virtual constructor();
    void virtual calculate();
    void virtual generatePath();
    void virtual initializeMap();
};


Dijstra::Dijstra(){
    start_point_pub = nh_.advertise<geometry_msgs::PointStamped>("/starting_point", 1);
    goal_point_pub = nh_.advertise<geometry_msgs::PointStamped>("/goal_point", 1);
}


// Dijstra specialized
Dijstra::~Dijstra(){
    for(int i = 0; i<width; i++)
        delete node_map[i];
    delete node_map;
}

// Dijstra specialized
double Dijstra::gFunction(int x1, int y1, int x2, int y2){
    double g = sqrt(pow(x1-x2,2) + pow(y1-y2, 2));

    return g;
}

// Global + Dijstra -> made virtual and call the parent's function
void Dijstra::constructor(){
    GlobalPlanner::constructor();

    // Dijstra specialized
    // map generation
    node_map = new NodeDijstra*[width];
    for(int i = 0; i< width; i++)
        node_map[i] = new NodeDijstra[height];

    // populating map
    for(int i = 0; i<width; i++){
        for(int j = 0; j<height; j++){
            node_map[i][j].set_position(i,j);
            if(map.data[coordinateToData(Coordinate(i,j))] == 0)
                node_map[i][j].set_occupied(false);
            else
                node_map[i][j].set_occupied(true);
        }
    }

    // algorithm start setting
    current_node = &node_map[start_.x_][start_.y_];
    current_node->set_gcost(0);
}


// Dijstra specialized
void Dijstra::initializeMap(){
    // empty open&closed list and path
    while(!open_list.empty())
        open_list.pop();
    closed_list.clear();
    path.poses.clear();
    // reset gcost
    for(int i = 0; i<width; i++){
        for(int j = 0; j<height; j++){
            node_map[i][j].set_gcost(INT_MAX);
            node_map[i][j].set_parent(nullptr);
        }
    }
    // algorithm start setting
    current_node = &node_map[start_.x_][start_.y_];
    current_node->set_gcost(0);
}

// Dijstra specialized
void Dijstra::calculate(){
    while(current_node != &node_map[goal_.x_][goal_.y_]){
        // visit all the neighbors
        for(int i = -1; i <= 1; i++){
            for(int j = -1; j <= 1; j++){
                int neighbor_x = current_node->get_coordinate().x_ + i;
                int neighbor_y = current_node->get_coordinate().y_ + j;
                // if the neighbor is out of boundary, or occupied, then skip
                if(neighbor_x < 0 || neighbor_x >= width || neighbor_y < 0 || neighbor_y >= height || (neighbor_x==0&&neighbor_y==0) || node_map[neighbor_x][neighbor_y].get_occupied() == true)
                    continue;
                double temp_g_cost = gFunction(current_node->get_coordinate().x_, current_node->get_coordinate().y_, neighbor_x, neighbor_y) + current_node->get_gcost();
                // if the node has a g_cost that is higher than the one proposing now, than we update it and add it in the open list
                if(node_map[neighbor_x][neighbor_y].get_gcost() > temp_g_cost){
                    node_map[neighbor_x][neighbor_y].set_gcost(temp_g_cost);
                    node_map[neighbor_x][neighbor_y].set_parent(current_node);
                    // even if the node is in open_list, because we updated the value, we insert again
                    open_list.push(&node_map[neighbor_x][neighbor_y]);
                }
            }
        }
        // choose next current_node, and insert it into closed list
        current_node = open_list.top();
        open_list.pop();

        // checking if it is already in closed_list, if so, find the best one that is not
        while(true){
            auto finder = closed_list.find(current_node);
            if (finder != closed_list.end()){
                // std::cout << "skipping.. already in closed list"<< std::endl;
                current_node = open_list.top();
                open_list.pop();
                continue;
            }else
                break;
        }
        closed_list.insert(std::make_pair(current_node, current_node->get_parent()));
    }
}


// Dijstra specialized
void Dijstra::generatePath(){
    // backtrack closed_list starting from goal node to start node
    std::queue<NodeDijstra*> temp_path;

    // backtrack closed_list starting from goal to start
    NodeDijstra* printer = &node_map[goal_.x_][goal_.y_];
    while(true){
        if(printer == 0){
            std::cout<<"cannot produce path!"<< std::endl;
            return;
        }
        if(printer == &node_map[start_.x_][start_.y_]){
            temp_path.push(printer);
            break;
        }        
        temp_path.push(printer);
        printer = closed_list[printer];
    }
    // flip and generate path
    while(!temp_path.empty()){
        geometry_msgs::PoseStamped path_pose;
        path_pose.header = header_;
        path_pose.pose = coordinateToPose(temp_path.front()->get_coordinate());
        temp_path.pop();
        path.poses.push_back(path_pose);
    }
}


// Global + Dijstra -> made virtual and call the parent's function
void Dijstra::publish(){
    GlobalPlanner::publish();
    geometry_msgs::PointStamped start;
    geometry_msgs::PointStamped goal;
    start.header = header_;
    goal.header = header_;
    start.point = coordinateToPose(start_).position;
    goal.point = coordinateToPose(goal_).position;

    start_point_pub.publish(start);
    goal_point_pub.publish(goal);
    // std::cout<<"successfully finished publishing path!"<<std::endl;
}
