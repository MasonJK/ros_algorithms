#pragma once
#include <iostream>
#include <queue>
#include <vector>
#include <unordered_map>
#include <ctime>
#include <climits>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

class Coordinate{
public:
    int x_;
    int y_;
    Coordinate(){x_ = 0; y_ = 0;}
    Coordinate(int x, int y){x_ = x; y_ = y;}
    bool operator==(Coordinate c){
        if((this->x_ == c.x_) && (this->y_ == c.y_))
            return true;
        else
            return false;
    }
    bool operator!=(Coordinate c){
        if((this->x_ == c.x_) && (this->y_ == c.y_))
            return false;
        else
            return true;
    }
};


class GlobalPlanner{
protected:
    ros::NodeHandle nh_;
    ros::Publisher path_pub;
    ros::Subscriber start_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber map_sub;

    nav_msgs::Path path;
    nav_msgs::OccupancyGrid map;
    int width, height;
    float resolution_;
    Coordinate start_, goal_;
    std_msgs::Header header_;
    geometry_msgs::Pose origin_;
    bool initial_constructor;
    bool initial_map_flag;
    bool initial_start_flag;
    bool initial_goal_flag;
public:
    GlobalPlanner();
    ~GlobalPlanner();

    int coordinateToData(Coordinate coordinate);
    Coordinate poseToCoordinate(geometry_msgs::Pose pose);
    geometry_msgs::Pose coordinateToPose(Coordinate coordinate);
    void startCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void run();
    void virtual publish();
    void virtual constructor();
    void virtual calculate() = 0;
    void virtual generatePath() = 0;
    void virtual initializeMap() = 0;
};


GlobalPlanner::GlobalPlanner(){
    // publisher, subscriber
    path_pub = nh_.advertise<nav_msgs::Path>("/dijstra_path", 1);
    start_sub = nh_.subscribe("odom", 1, &GlobalPlanner::startCallback, this);
    goal_sub = nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlanner::goalCallback, this);
    map_sub = nh_.subscribe("/map", 1, &GlobalPlanner::mapCallback, this);

    initial_map_flag = false;
    initial_constructor = true;
    initial_start_flag = false;
    initial_goal_flag = false;
    start_ = Coordinate();
    goal_ = Coordinate();
}

GlobalPlanner::~GlobalPlanner(){}


void GlobalPlanner::startCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if(start_ != poseToCoordinate(msg->pose.pose)){
        std::cout<<"got new starting point!"<<std::endl;
        // new_start_flag = true;
    }
    start_ = poseToCoordinate(msg->pose.pose);
    initial_start_flag = 
    true;
}


void GlobalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(goal_ != poseToCoordinate(msg->pose)){
        std::cout<<"got new goal point!"<<std::endl;
        // new_goal_flag = true;
    }
    goal_ = poseToCoordinate(msg->pose);
    initial_goal_flag = true;
}


void GlobalPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map = *msg;
    initial_map_flag = true;
}


int GlobalPlanner::coordinateToData(Coordinate coordinate){
    return (coordinate.y_-1)*width + coordinate.x_;
}


Coordinate GlobalPlanner::poseToCoordinate(geometry_msgs::Pose pose){
    Coordinate coordinate;
    coordinate.x_ = int(round((pose.position.x - origin_.position.x)/resolution_));
    coordinate.y_ = int(round((pose.position.y - origin_.position.y)/resolution_));
    return coordinate;
}


geometry_msgs::Pose GlobalPlanner::coordinateToPose(Coordinate coordinate){
    geometry_msgs::Pose pose;
    pose.position.x = (coordinate.x_ * resolution_) + origin_.position.x;
    pose.position.y = (coordinate.y_ * resolution_) + origin_.position.y;
    return pose;
}

void GlobalPlanner::constructor(){
    width = map.info.width;
    height = map.info.height;
    header_.seq = 0;
    header_.frame_id = map.header.frame_id;
    origin_ = map.info.origin;
    resolution_ = map.info.resolution;
}

void GlobalPlanner::run(){
    clock_t start, finish;
    double duration;
    start = clock();

    if(initial_map_flag){                    // is map received?
        if(initial_constructor){             // constructor is not called at all
            constructor();
            initial_constructor = false;     // constructor will not be called again
        }
        header_.seq++;
        header_.stamp = ros::Time::now();

        if(initial_start_flag && initial_goal_flag){
            initializeMap();
            calculate();
            generatePath();
        }
        publish();
    }
    finish = clock();

    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    std::cout<< "runtime: " << duration << " seconds" << std::endl;
}

void GlobalPlanner::publish(){
    path.header = header_;
    path_pub.publish(path);
}
