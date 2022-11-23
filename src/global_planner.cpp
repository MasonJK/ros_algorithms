#include <ros_algorithms/dijstra.h>
#include <ros_algorithms/a_star.h>
#include <ros_algorithms/global_planner.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle nh;

    GlobalPlanner *global_planner;

    // global_planner = new Dijstra;
    // std::cout<<"Using Dijstra Algorithm!"<<std::endl;
    
    global_planner = new AStar;
    std::cout<<"Using AStar Algorithm!"<<std::endl;

    while(ros::ok()){
        ros::spinOnce();
        global_planner->run();
    }

    delete global_planner;
    return 0;
}