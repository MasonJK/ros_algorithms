#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


class odomPublisher{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double x,y,th, vx, vy, vth;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;

public:
    odomPublisher(){
        x = -7.19367599487;
        y = -0.85893535614;
        th = 0.0;

        vx = 0.0;
        vy = 0.0;
        vth = 0.0;

        pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
        sub_ = nh_.subscribe("cmd_vel", 1, &odomPublisher::callback, this);
    }

    void callback(const geometry_msgs::Twist::ConstPtr& msg){
        vx = msg->linear.x;
        vy = msg->linear.y;
        vth = msg->angular.z;
    }

    void update(){
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "map";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        pub_.publish(odom);
        last_time = current_time;
    }

};


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  odomPublisher odom_publisher;

  while(ros::ok()){
    ros::spinOnce();
    odom_publisher.update();
  }
}