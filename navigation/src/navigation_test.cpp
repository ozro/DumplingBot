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
    float cur_x,cur_y,cur_ang;
    float cur_goal_x,cur_goal_y,cur_goal_ang;
    float del_x,del_y,del_ang;
    float max_speed_linear;
    float max_speed_angular;
    float stop_distance_x,stop_distance_y,stop_distance_ang;
    float feed_forward_x,feed_forward_y,feed_forward_ang;
    int cur_goal_index;
    float threshold_x,threshold_y,threshold_ang;
    std::vector<geometry_msgs::Vector3> path;
    std::vector<geometry_msgs::Vector3> test_path;

  public:
    ros::Time last_message_received;
    void init(){
      odom_sub = n_.subscribe("/odometry/filtered_map",10,&navigation::odom_callback,this);
     // goal_sub = n_.subscribe("/",100,&navigation::odom_callback,this);
      command_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel",10);
      plan_client = n_.serviceClient<planning::plan>("plan_path");
      max_speed_linear = 0.25;
      max_speed_angular = 0.3;
      stop_distance_x = 0.5;
      stop_distance_y = 0.5;
      stop_distance_ang = 1;
      feed_forward_x = 0.15;
      feed_forward_y = 0.55;
      feed_forward_ang = 0.2;
      cur_goal_index = 0;
      threshold_x = 0.03;
      threshold_y = 0.03;
      threshold_ang = 0.05;
      geometry_msgs::Vector3 path_pose;
      path_pose.x = 1.0;
      path_pose.y = 0;
      path_pose.z = 0;
      path.push_back(path_pose);
      path_pose.x = 1,0;
      path_pose.y = 0;
      path_pose.z = 0;
      path.push_back(path_pose);
      ROS_INFO("size :%d", path.size());

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

int get_sign(const float goal, const float cur){
  if (goal - cur >0){
      return 1;
    }else{
      return -1;
    }
}
float get_velocity(float del, float max_speed, float threshold, float stop_distance, int sign, float feed_forward){
  float velocity;
  if(fabs(del) > stop_distance){
    //need to subscribe to current speed!!!!!!!!!
    velocity = sign*max_speed;
  }else{
    velocity = sign*(max_speed*fabs(del)/stop_distance+feed_forward);
  }
  if(fabs(del) <threshold){
    velocity =0;
  }
  return velocity;
}
void navigation::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_message){
  /*need ro change to navigate odom type*/


    if(!path.empty()){
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
      cur_goal_x = path[cur_goal_index].x;
      cur_goal_y = path[cur_goal_index].y;
      cur_goal_ang = path[cur_goal_index].z;
      ROS_INFO("cur_goal_x, %f",cur_goal_x);
      ROS_INFO("cur_goal_y, %f",cur_goal_y);
      ROS_INFO("cur_goal_yaw, %f",cur_goal_ang);

      //cur_goal_x = 0.45;
      //cur_goal_y = 0.11;
      //cur_goal_ang = -0.06;
      float del_x = cur_goal_x-cur_x;
      float del_y = cur_goal_y-cur_y;
      float del_ang = cur_goal_ang-cur_ang;
      
      if(fabs(del_x)<threshold_x && fabs(del_y)<threshold_y && fabs(del_ang)<threshold_ang){
        
        cur_goal_index += 1;
        if(cur_goal_index<path.size()){
          cur_goal_x = path[cur_goal_index].x;
          cur_goal_y = path[cur_goal_index].y;
          cur_goal_ang = path[cur_goal_index].z;

        }else{
          ROS_INFO("reached one goal");
          ROS_INFO("path size,%d",path.size());
          ROS_INFO("cur_goal_index,%d",cur_goal_index);
          geometry_msgs::Twist cmd;
          command_pub.publish(cmd);
          ros::shutdown();
        }
      }
      /*
      if(cur_goal_index == path.size()){
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.angular.z = 0;
        command_pub.publish(cmd);
        ROS_INFO("reached final destination");
        ros::Duration(1111).sleep();
      }*/
      //ROS_INFO("%f, %f, %f", cur_goal_x,cur_goal_y,cur_goal_ang);
      int sign_x = get_sign(cur_goal_x,cur_x);
      int sign_y = get_sign(cur_goal_y,cur_y);
      int sign_ang = -get_sign(cur_goal_ang,cur_ang);
      geometry_msgs::Twist cmd;
      cmd.linear.x = get_velocity(del_x,max_speed_linear,threshold_x,stop_distance_x,sign_x,feed_forward_x);
      cmd.linear.y = get_velocity(del_y,max_speed_linear,threshold_y,stop_distance_y,sign_y,feed_forward_y);
      cmd.angular.z = get_velocity(del_ang,max_speed_angular,threshold_ang,stop_distance_ang,sign_ang,feed_forward_ang);
      //cmd.linear.y =0;
      ROS_INFO("cmd_x, %f",cmd.linear.x);
      ROS_INFO("cmd_y, %f",cmd.linear.y);
      ROS_INFO("cmd_yaw, %f",cmd.angular.z);
      command_pub.publish(cmd);
    }
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
