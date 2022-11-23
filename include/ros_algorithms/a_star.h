#pragma once
#include <ros_algorithms/dijstra.h>
#include <typeinfo>

class NodeAStar : public NodeDijstra{
protected:
    double h_cost_;
    NodeAStar* parent_;
public:
    NodeAStar(){
        h_cost_ = INT_MAX;
        parent_ = nullptr;
    }
    void set_parent(NodeAStar* parent){this->parent_ = parent;}
    void set_hcost(double h_cost){ h_cost_ = h_cost; }
    NodeAStar* get_parent(){return parent_;}
    double get_hcost(){return h_cost_;}
};

struct cmpAStar{
    bool operator()(NodeAStar* n1, NodeAStar* n2){
        if((n1->get_gcost() + n1->get_hcost()) >= (n2->get_gcost() + n2->get_hcost()))
            return true;
        else
            return false;
    }
};

class AStar : public GlobalPlanner {
protected:
    ros::Publisher start_point_pub;
    ros::Publisher goal_point_pub;

    NodeAStar** node_map;
    NodeAStar* current_node;
    std::priority_queue<NodeAStar*, std::vector<NodeAStar*>, cmpAStar> open_list;
    // NodeAStar가 key가 되고, parent NodeAStar가 value가 되는 closed_list
    std::unordered_map<NodeAStar*, NodeAStar*> closed_list;
    
public:
    AStar();
    ~AStar();
    double gFunction(int x1, int y1, int x2, int y2);
    double hFunction(int x1, int y1);
    void virtual publish();
    void virtual constructor();
    void virtual calculate();
    void virtual generatePath();
    void virtual initializeMap();
};

AStar::AStar(){
    start_point_pub = nh_.advertise<geometry_msgs::PointStamped>("/starting_point", 1);
    goal_point_pub = nh_.advertise<geometry_msgs::PointStamped>("/goal_point", 1);
}


AStar::~AStar(){
    for(int i = 0; i<width; i++)
        delete node_map[i];
    delete node_map;
}


double AStar::gFunction(int x1, int y1, int x2, int y2){
    double g = sqrt(pow(x1-x2,2) + pow(y1-y2, 2));

    return g;
}

double AStar::hFunction(int x1, int y1){
    double h = sqrt(pow(x1-goal_.x_ ,2) + pow(y1-goal_.x_, 2));

    return h;
}

void AStar::constructor(){
    GlobalPlanner::constructor();

    
    // map generation
    node_map = new NodeAStar*[width];
    for(int i = 0; i< width; i++)
        node_map[i] = new NodeAStar[height];

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
    current_node->set_hcost(hFunction(current_node->get_coordinate().x_, current_node->get_coordinate().y_));
}



void AStar::initializeMap(){
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
    current_node= &node_map[start_.x_][start_.y_];
    current_node->set_gcost(0);
    current_node->set_hcost(hFunction(current_node->get_coordinate().x_, current_node->get_coordinate().y_));
}


// same with Dijstra
void AStar::calculate(){
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
                // if the NodeAStar has a g_cost that is higher than the one proposing now, than we update it and add it in the open list
                if(node_map[neighbor_x][neighbor_y].get_gcost() > temp_g_cost){
                    node_map[neighbor_x][neighbor_y].set_gcost(temp_g_cost);
                    node_map[neighbor_x][neighbor_y].set_parent(current_node);
                    // even if the NodeAStar is in open_list, because we updated the value, we insert again
                    open_list.push(&node_map[neighbor_x][neighbor_y]);
                }
            }
        }
        // choose next current_NodeAStar, and insert it into closed list
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


// same with Dijstra
void AStar::generatePath(){
    // backtrack closed_list starting from goal NodeAStar to start NodeAStar
    std::queue<NodeAStar*> temp_path;

    // backtrack closed_list starting from goal to start
    NodeAStar* printer = &node_map[goal_.x_][goal_.y_];
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


// same with Dijstra
void AStar::publish(){
    GlobalPlanner::publish();
    geometry_msgs::PointStamped start;
    geometry_msgs::PointStamped goal;
    start.header = header_;
    goal.header = header_;
    start.point = coordinateToPose(start_).position;
    goal.point = coordinateToPose(goal_).position;

    start_point_pub.publish(start);
    goal_point_pub.publish(goal);
}