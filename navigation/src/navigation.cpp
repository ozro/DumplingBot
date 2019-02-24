#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h> 
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
    float stop_distance;
    int cur_goal_index;
    float threshold_x;
    float threshold_y;
    float threshold_ang;
    std::vector<geometry_msgs::Vector3> path;

  public:
    
    void init(){
      odom_sub = n_.subscribe("/odom",100,&navigation::odom_callback,this);
     // goal_sub = n_.subscribe("/",100,&navigation::odom_callback,this);
      command_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel",10);
      plan_client = n_.serviceClient<planning::plan>("plan_path");
      max_speed = 0.1;
      stop_distance = 0.5;
      cur_goal_index = 0;
      threshold_x = 0.1;
      threshold_y = 0.1;
      threshold_ang = 1;

      ros::NodeHandle n_private("~");
      get_global_plan(2,16);


    }
    ~navigation(){}
    void get_global_plan(int start, int goal);
    void odom_callback(const geometry_msgs::Vector3::ConstPtr& message);
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

void navigation::odom_callback(const geometry_msgs::Vector3::ConstPtr& odom_message){
    if(! path.empty()){
      cur_x = odom_message->x;
      cur_y = odom_message->y;
      cur_ang = odom_message->z;
      
      cur_goal_x = path[cur_goal_index].x;
      cur_goal_y = path[cur_goal_index].y;
      cur_goal_ang = path[cur_goal_index].z;

      if(fabs(cur_x-cur_goal_x)<threshold_x && fabs(cur_y-cur_goal_y)<threshold_y && fabs(cur_ang-cur_goal_ang)<threshold_x
         && cur_goal_index<path.size()){
        cur_goal_index += 1;
        cur_goal_x = path[cur_goal_index].x;
        cur_goal_y = path[cur_goal_index].y;
        cur_goal_ang = path[cur_goal_index].z;
        ROS_INFO("reached one goal");
      }
      ROS_INFO("%f, %f, %f", cur_goal_x,cur_goal_y,cur_goal_ang);

      int sign = 1;
      geometry_msgs::Twist cmd;

      if (cur_goal_x - cur_x >0){
        sign = 1;
      }else{
        sign = -1;
      }
      if(fabs(cur_x-cur_goal_x) > stop_distance){
        //need to subscribe to current speed!!!!!!!!!
        cmd.linear.x = sign*max_speed;
      }else{
        cmd.linear.x = sign*max_speed*(cur_x-cur_goal_x)/stop_distance;
      }
      if (cur_goal_y - cur_y >0){
        sign = 1;
      }else{
        sign = -1;
      }
      if(fabs(cur_y-cur_goal_y) > stop_distance){
        //need to subscribe to current speed!!!!!!!!!
        cmd.linear.y = sign*max_speed;
      }else{
        cmd.linear.y = sign*max_speed*(cur_y-cur_goal_y)/stop_distance;
      }
      if (cur_goal_ang - cur_ang >0){
        sign = -1;
      }else{
        sign = 1;
      }
      if(fabs(cur_ang-cur_goal_ang) > stop_distance){
        //need to subscribe to current speed!!!!!!!!!
        cmd.angular.z = sign*max_speed*5;
      }else{
        cmd.angular.z = sign*max_speed*5*(cur_ang-cur_goal_ang)/stop_distance;
      }
      command_pub.publish(cmd);
    }

}


int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
    ros::init(argc, argv,"navigation");
    navigation server;
    server.init();
    while(ros::ok()){
      //server.send_vel();
      ros::spinOnce();
      ros::Duration(0.1).sleep();

    }
    return 0;
}
