#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <fstream>
#include <std_msgs/Int16MultiArray.h>
#include <eigen3/Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
double pi = 3.1415926535897932385;
class localization{
  private:
    ros::NodeHandle n_;
    ros::Subscriber encoder_sub;
    ros::Publisher command_pub;
    ros::Publisher odom_pub;
    std::vector<std::vector<float> > map;
    Eigen::Quaterniond cur_q;
    geometry_msgs::Vector3 cur_pose;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    float base_width;
    float base_length;
    float wheel_gap;
    float wheel_setback;
    float wheel_radius;
    float vx;
    float vy;
    float vth;
    float deltaFL;
    float deltaFR;
    float deltaBL;
    float deltaBR;
    int prev_FL;
    int prev_FR;
    int prev_BL;
    int prev_BR;
    double x;
    double y;
    double th;
    ros::Time last_time;

  public:
    
    void init(){
      encoder_sub = n_.subscribe("/encoder",100,&localization::encoder_callback,this);
      odom_pub = n_.advertise<nav_msgs::Odometry>("encoder_odom",10);
      ros::NodeHandle n_private("~");
      base_width = 0.5;
      base_length = 0.5;
      wheel_gap = 0.0508;
      wheel_setback = 0.041275;
      wheel_radius = 0.1016; 
      prev_FL = 0;
      prev_FR = 0;
      prev_BL = 0;
      prev_BR = 0;
      x = 0;
      y = 0;
      th = 0;
      last_time = ros::Time::now();

    }
    ~localization(){}
    void encoder_callback(const std_msgs::Int16MultiArray::ConstPtr& message);
};

void localization::encoder_callback(const std_msgs::Int16MultiArray::ConstPtr& message){
  ros::Time current_time = ros::Time::now();
  double DistancePerCount = (3.14159265 * wheel_radius*2) / 2626; 
  double lengthBetweenTwoWheels = 0.25;
  float DISTANCE_LEFT_TO_RIGHT_WHEEL = this->base_width +2*this->wheel_gap;
  float DISTANCE_FRONT_TO_REAR_WHEEL = this->base_length -2*this->wheel_setback;
  float WHEEL_SEPARATION_WIDTH = DISTANCE_LEFT_TO_RIGHT_WHEEL / 2;
  float WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2;
  int tick_FL = message->data[0];
  int tick_FR = message->data[1];
  int tick_BL = message->data[2];
  int tick_BR = message->data[3];
  double dt = (current_time - last_time).toSec();
  //extract the wheel velocities from the tick signals count
  deltaFL = (tick_FL - prev_FL)*DistancePerCount/dt;
  deltaFR = (tick_FR - prev_FR)*DistancePerCount/dt;
  deltaBL = (tick_BL - prev_BL)*DistancePerCount/dt;
  deltaBR = (tick_BR - prev_BR)*DistancePerCount/dt;
  
  vx = (deltaFL+deltaFR+deltaBL+deltaBR)/4;
  vy = (-deltaFL+deltaFR+deltaBL-deltaBR)/4;
  vth = ( -deltaFL + deltaFR - deltaBL + deltaBR) * (1/(4 * (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)));

  
  
  double delta_x =  vx * dt;
  double delta_y =  vy * dt;
  double delta_th = vth * dt;
  ROS_INFO("dx: %f",delta_x);
  x += delta_x;
  y += delta_y;
  th += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //Odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

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

  //publish the message
  odom_pub.publish(odom);
  prev_FL = tick_FL;
  prev_FR = tick_FR;
  prev_BL = tick_BL;
  prev_BR = tick_BR;

  last_time = current_time;

}


int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
    ros::init(argc, argv,"localization");
    localization server;
    server.init();
    while(ros::ok()){
      ros::spinOnce();
      ros::Duration(0.1).sleep();

    }
    return 0;
}
