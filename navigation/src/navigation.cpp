#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h> 
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Geometry>
#include "planning/plan.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

class navigation{
	private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub;
    ros::ServiceClient plan_client;
    //ros::Subscriber goal_sub;
    ros::Publisher command_pub;
    float cur_x;
    float cur_y;
    float cur_ang;
    float cur_goal_x;
    float cur_goal_y;
    float cur_goal_ang;
    float max_speed;
    float max_speed_angular;
    float stop_distance;
    int cur_goal_index;
    float threshold_x;
    float threshold_y;
    float threshold_ang;
    std::vector<geometry_msgs::Vector3> path;

  public:
    ros::Time last_message_received;
    void init(){
      odom_sub = n_.subscribe("/odom",10,&navigation::odom_callback,this);
     // goal_sub = n_.subscribe("/",100,&navigation::odom_callback,this);
      command_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel",10);
      plan_client = n_.serviceClient<planning::plan>("plan_path");
      max_speed = 0.2;
      max_speed_angular = 0.05;
      stop_distance = 0.5;
      cur_goal_index = 0;
      threshold_x = 0.1;
      threshold_y = 0.1;
      threshold_ang = 0.1;

      ros::NodeHandle n_private("~");
      //get_global_plan(2,29);


    }
    ~navigation(){}
    void get_global_plan(int start, int goal);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& message);
    void stop_motion();
};
void navigation::get_global_plan(int start, int goal){
  planning::plan get_plan;
  get_plan.request.start = start;
  get_plan.request.goal = goal;
  if (plan_client.call(get_plan))
  { 
    path = get_plan.response.path;
  }
  else
  {
    ROS_ERROR("Failed to call service get_plan");
  }

}
void navigation::stop_motion(){
  geometry_msgs::Twist cmd;
  command_pub.publish(cmd);
}
void navigation::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_message){
  /*need ro change to navigate odom type*/


    //if(! path.empty()){
      cur_x = odom_message->pose.pose.position.x;
      cur_y = odom_message->pose.pose.position.y;
      tf::Quaternion q(
          odom_message->pose.pose.orientation.x,
          odom_message->pose.pose.orientation.y,
          odom_message->pose.pose.orientation.z,
          odom_message->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

     // cur_ang = odom_message->z;
      cur_ang = yaw;

      ROS_INFO("cur_x, %f",cur_x);
      ROS_INFO("cur_y, %f",cur_y);
      ROS_INFO("cur_yaw, %f",yaw);
    //  cur_goal_x = path[cur_goal_index].x;
     // cur_goal_y = path[cur_goal_index].y;
     // cur_goal_ang = path[cur_goal_index].z/180*3.14;

      cur_goal_x = 1.55;
      cur_goal_y = 0.12;
      cur_goal_ang = 1;


/*
      float del_x = cur_goal_x-cur_x;
      float del_y = cur_goal_y-cur_y;
      float del_ang = atan2(del_y,del_x);
      float dist = sqrt(del_x*del_x+del_y*del_y);
      float x_dist = dist*cos(del_ang);
      float y_dist = dist*sin(del_ang);*/
      //ROS_INFO("%f, %f, %f", x_dist,y_dist,del_ang);

      /*
      if(fabs(cur_x-cur_goal_x)<threshold_x && fabs(cur_y-cur_goal_y)<threshold_y && fabs(cur_ang-cur_goal_ang)<threshold_ang
         && cur_goal_index<path.size()){
        cur_goal_index += 1;
        cur_goal_x = path[cur_goal_index].x;
        cur_goal_y = path[cur_goal_index].y;
        cur_goal_ang = path[cur_goal_index].z;
        ROS_INFO("reached one goal");
      }*/
      
      /*if(cur_goal_index == path.size()){
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.angular.z = 0;
        command_pub.publish(cmd);
        ros::Duration(1111).sleep();
      }*/
      //ROS_INFO("%f, %f, %f", cur_goal_x,cur_goal_y,cur_goal_ang);

      int sign_x = 1;
      int sign_y = 1;
      int sign_ang = 1;
      geometry_msgs::Twist cmd;

      if (cur_goal_x - cur_x >0){
        sign_x = 1;
      }else{
        sign_x = -1;
      }
      if(fabs(cur_x-cur_goal_x) > stop_distance){
        //need to subscribe to current speed!!!!!!!!!
        cmd.linear.x = sign_x*max_speed;
      }else{
        cmd.linear.x = sign_x*(max_speed*fabs(cur_goal_x-cur_x)/stop_distance+0.3);
      }
      if(fabs(cur_x-cur_goal_x) <threshold_x){
        cmd.linear.x =0;
      }
      if (cur_goal_y - cur_y >0){
        sign_y = 1;
      }else{
        sign_y = -1;
      }
      if(fabs(cur_y-cur_goal_y) > stop_distance){
        //need to subscribe to current speed!!!!!!!!!
        cmd.linear.y = sign_y*max_speed;
      }else{
        cmd.linear.y = sign_y*(max_speed*fabs(cur_goal_y-cur_y)/stop_distance+0.6);
      }
      if(fabs(cur_y-cur_goal_y) <threshold_y){
        cmd.linear.y =0;
      }
      if (cur_goal_ang - cur_ang >0){
        sign_ang = -1;
      }else{
        sign_ang = 1;
      }
      if(fabs(cur_ang-cur_goal_ang) > 1){
        //need to subscribe to current speed!!!!!!!!!
        cmd.angular.z = 0.4*sign_ang;
      }else{
        cmd.angular.z = sign_ang*(0.4*fabs(cur_goal_ang-cur_ang)/1+0.2);
      }
      if(fabs(cur_ang-cur_goal_ang) <threshold_ang){
        cmd.angular.z =0;
      }
      ROS_INFO("x speed, %f",cmd.linear.x);
      ROS_INFO("y speed, %f",cmd.linear.y);
      ROS_INFO("yaw speed, %f",cmd.angular.z);
      //cmd.linear.x = 0;
      //cmd.linear.y = 0;
      //cmd.angular.z = 0;

      command_pub.publish(cmd);
    //}
    this->last_message_received = ros::Time::now();

}


int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
    ros::init(argc, argv,"navigation");
    navigation server;
    server.init();
    while(ros::ok()){
      //server.send_vel();
      //ROS_INFO("aaaaaa");
      ros::spinOnce();
      ros::Duration(0.1).sleep();
      ros::Time current = ros::Time::now();

      
      ros::Duration interval = current- server.last_message_received;
      if(interval.toSec()>100000000){
        server.stop_motion();
      }
    }
    return 0;
}
