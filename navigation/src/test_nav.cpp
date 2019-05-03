#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h> 
#include <std_msgs/Int16MultiArray.h> 
#include <std_msgs/String.h> 
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Geometry>
#include "planning/plan.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
const double pi = 3.14159;
class navigation{
  private:
    ros::NodeHandle n_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher state_pub_;
    tf::TransformListener listener_;
    ros::Subscriber odom_sub;
    ros::Subscriber go_sub;
    float cur_x,cur_y,cur_ang;
    float cur_goal_x,cur_goal_y,cur_goal_ang;
    float del_x,del_y,del_ang;
    float max_speed_linear;
    float max_speed_angular;
    float stop_distance_x,stop_distance_y,stop_distance_ang;
    float feed_forward_x,feed_forward_y,feed_forward_ang;
    int cur_goal_index;
    bool called;
    bool reached_waypoint;
    bool reached_final_goal;
    bool reached_x;
    bool reached_ang;
    bool controlled;
    bool docking;
    bool docked;
    bool go;
    bool running;
    float threshold_x,threshold_y,threshold_ang;
    std::vector<geometry_msgs::Vector3> path;
    ros::ServiceClient plan_client;

  public:
    void init(){
      odom_sub = n_.subscribe("/odometry/filtered_map",1,&navigation::odom_callback,this);
      go_sub = n_.subscribe("/navigate",1,&navigation::navigate_callback,this);
      plan_client = n_.serviceClient<planning::plan>("plan_path");
      cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      state_pub_ = n_.advertise<std_msgs::String>("Transit_status", 10);
     // goal_sub = n_.subscribe("/",100,&navigation::odom_callback,this);
      go = false;
      running = false;
      called = false;
      reached_waypoint = false;
      reached_final_goal = false;
      reached_x = false;
      reached_ang = false;
      controlled = false;
      docking = false;
      docked = false;
      max_speed_linear = 0.25;
      max_speed_angular = 0.35;
      stop_distance_x = 0.5;
      stop_distance_y = 0.5;
      stop_distance_ang = 1;
      feed_forward_x = 0.15;
      feed_forward_y = 0.56;
      feed_forward_ang = 0.27;
      cur_goal_index = 0;
      threshold_x = 0.03;
      threshold_y = 0.03;
      threshold_ang = 0.03;
      //get_global_plan(0,24);
      geometry_msgs::Vector3 path_pose;
      /*path_pose.x = 2.77;
      path_pose.y = 0;
      path_pose.z = 0;
      path.push_back(path_pose);
      path_pose.x = 2.77;
      path_pose.y = 0;
      path_pose.z = -1.57;
      path.push_back(path_pose);
      path_pose.x = 2.77;
      path_pose.y = -1;
      path_pose.z = -1.57;
      path.push_back(path_pose);
      path_pose.x = 2.77;
      path_pose.y = -2;
      path_pose.z = -1.57;
      path.push_back(path_pose);
      path_pose.x = 2.77;
      path_pose.y = -2.7;
      path_pose.z = -1.57;
      path.push_back(path_pose);
      path_pose.x = 2.74;
      path_pose.y = -2.77;
      path_pose.z = -3.14;
      path.push_back(path_pose);
      path_pose.x = 1.33;
      path_pose.y = -2.77;
      path_pose.z = -3.14;
      path.push_back(path_pose);
      path_pose.x = 0.33;
      path_pose.y = -2.77;
      path_pose.z = -3.14;
      path.push_back(path_pose);
      path_pose.x = -0.37;
      path_pose.y = -2.74;
      path_pose.z = -3.14;
      path.push_back(path_pose);*/
      path_pose.x = -0.37;
      path_pose.y = -2.77;
      path_pose.z = -3.14;
      //path.push_back(path_pose);

      //tag8 2.75,-2.84,-3.1
      /*path_pose.x = -0.38;
      path_pose.y = -2.79;
      path_pose.z = 1.57;

      path.push_back(path_pose);*/
      //cur_goal_x = path[cur_goal_index].x;
      //cur_goal_y = path[cur_goal_index].y;
      //cur_goal_ang = path[cur_goal_index].z;
      
      //ROS_INFO("aaaaacur_goal_x:%f, cur_goal_y:%f, cur_goal_z:%f",cur_goal_x,cur_goal_y,cur_goal_ang);
      /*path_pose.x = 1.2;
      path_pose.y = -0.65;
      path_pose.z = -1.57;
      path.push_back(path_pose);
      path_pose.x = 1.2;
      path_pose.y = -1.3;
      path_pose.z = -1.57;
      path.push_back(path_pose);*/
      
      ros::NodeHandle n_private("~");
      //get_global_plan(2,29);


    }
    ~navigation(){}
    void odom_callback(const nav_msgs::Odometry::ConstPtr& message);
    void navigate_callback(const std_msgs::Int16MultiArray::ConstPtr& message);
    void stop();
    bool driveForwardOdom(double distance);
    bool turnOdom(bool clockwise, double radians);
    void get_global_plan(int start, int goal);
    bool controller();

    //void docking(float goal_x,float goal_y,float goal_ang);
};

int get_sign(const float del){
  if (del > 0){
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
/*void navigation::docking(float goal_x,float goal_y,float goal_ang){
  ros::Rate rate(10.0);
  del_x = goal_x-cur_x;
  del_y = goal_y-cur_y;
  del_ang = goal_ang-cur_ang;
  while(fabs(del_x)>threshold_x || fabs(del_y)>threshold_y || fabs(del_ang)>threshold_ang && n_.ok()){
    rate.sleep();
    ros::spinOnce();
    ROS_INFO("del_x:%f, del_y:%f, del_z:%f",del_x,del_y,del_ang);
    
  }
  
}*/
void navigation::get_global_plan(int start, int goal){
  planning::plan get_plan;
  get_plan.request.start = start;
  get_plan.request.goal = goal;
  if (plan_client.call(get_plan))
  { 
    path = get_plan.response.path;
    ROS_INFO("AAA");
  }
  else
  {
    ROS_ERROR("Failed to call service get_plan");
  }

}
void navigation::navigate_callback(const std_msgs::Int16MultiArray::ConstPtr& message){
  if(running == false){
    if (message->data.size()>1){
      int start = message->data[0];
      int goal = message->data[1];
      ROS_INFO("%d",start);
      ROS_INFO("%d",goal);
      get_global_plan(start,goal);
      cur_goal_index = 0;
      go = true;
      running = true;
      reached_final_goal = false;
      reached_waypoint = false;
    }
    

  }

}
void navigation::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_message){
  if(running == true){
    called = true;
    /*need ro change to navigate odom type*/
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
    cur_ang = yaw;
    ROS_INFO("cur_x:%f, cur_y:%f, cur_z:%f",cur_x,cur_y,cur_ang);
    if(controlled == false){
      
      controller();
    }
    if(docking){
      ROS_INFO("cur_goal_x:%f, cur_goal_y:%f, cur_goal_z:%f",cur_goal_x,cur_goal_y,cur_goal_ang);
      del_x = cur_goal_x-cur_x;
      del_y = cur_goal_y-cur_y;
      del_ang = cur_goal_ang-cur_ang;
      if(cur_ang>-pi/4 &&cur_ang <=pi/4){
        del_x = cur_goal_x-cur_x;
        del_y = cur_goal_y-cur_y;
      }
      //right
      if(cur_ang<=-pi/4 &&cur_ang >-3*pi/4){
        del_x = -(cur_goal_y-cur_y);
        del_y = cur_goal_x-cur_x;
      }
      //down
      if(cur_ang<=-3*pi/4 ||cur_ang >3*pi/4){
        del_x = -(cur_goal_x-cur_x);
        del_y = -(cur_goal_y-cur_y);
        
      }
      //left
      if(cur_ang>pi/4 &&cur_ang <=3*pi/4){
        del_x = cur_goal_y-cur_y;
        del_y = -(cur_goal_x-cur_x);
      }
      if (fabs(del_ang)>pi){
        if (del_ang >0){
          del_ang = del_ang - 2*pi;
        }else{
          del_ang = del_ang + 2*pi;
        }
        ROS_INFO("del_ang!!!!!!!!!!!:%f,%f",del_ang,cur_ang);
      }
      
      if(fabs(del_x)>threshold_x || fabs(del_y)>threshold_y || fabs(del_ang)>threshold_ang){

        int sign_x = get_sign(del_x);
        int sign_y = get_sign(del_y);
        int sign_ang = get_sign(del_ang);
        geometry_msgs::Twist cmd;
        cmd.linear.x = get_velocity(del_x,max_speed_linear,threshold_x,stop_distance_x,sign_x,feed_forward_x);
        cmd.linear.y = get_velocity(del_y,max_speed_linear,threshold_y,stop_distance_y,sign_y,feed_forward_y);
        cmd.angular.z = get_velocity(del_ang,max_speed_angular,threshold_ang,stop_distance_ang,sign_ang,feed_forward_ang);
        ROS_INFO("Docking");
        ROS_INFO("del_x:%f, del_y:%f, del_z:%f",del_x,del_y,del_ang);
        ROS_INFO("cmd_x:%f, cmd_y:%f, cmd_z:%f",cmd.linear.x,cmd.linear.y,cmd.angular.z);
        cmd_vel_pub_.publish(cmd);
      }else{
        ROS_INFO("Doked already!!!!!!!!");
        docked = true;
      }
      
    }
  }
  
}

bool navigation::controller(){
  
  controlled = true;
  if(called== true && running == true && path.size()>0){
    cur_goal_x = path[cur_goal_index].x;
    cur_goal_y = path[cur_goal_index].y;
    cur_goal_ang = path[cur_goal_index].z;
    ROS_INFO("cur_goal_x:%f, cur_goal_y:%f, cur_goal_z:%f",cur_goal_x,cur_goal_y,cur_goal_ang);
    del_x = cur_goal_x-cur_x;
    del_y = cur_goal_y-cur_y;
    del_ang = cur_goal_ang-cur_ang;
    // up
    if(cur_ang>-pi/4 &&cur_ang <=pi/4){
      del_x = cur_goal_x-cur_x;
      del_y = cur_goal_y-cur_y;
    }
    //right
    if(cur_ang<=-pi/4 &&cur_ang >-3*pi/4){
      del_x = -(cur_goal_y-cur_y);
      del_y = cur_goal_x-cur_x;
    }
    //down
    if(cur_ang<=-3*pi/4 &&cur_ang >3*pi/4){
      del_x = -(cur_goal_x-cur_x);
      del_y = -(cur_goal_y-cur_y);
    }
    //left
    if(cur_ang>pi/4 &&cur_ang <=3*pi/4){
      del_x = cur_goal_y-cur_y;
      del_y = -(cur_goal_x-cur_x);
    }
    ROS_INFO("del_x:%f, del_y:%f, del_z:%f",del_x,del_y,del_ang);
    ROS_INFO("cur_x:%f, cur_y:%f, cur_z:%f",cur_x,cur_y,cur_ang);
    ros::Rate rate(5.0);
    while(!reached_final_goal){
      ros::spinOnce();

      del_x = cur_goal_x-cur_x;
      del_y = cur_goal_y-cur_y;
      del_ang = cur_goal_ang-cur_ang;
      // up
      if(cur_ang>-pi/4 &&cur_ang <=pi/4){
        del_x = cur_goal_x-cur_x;
        del_y = cur_goal_y-cur_y;
      }
      //right
      if(cur_ang<=-pi/4 &&cur_ang >-3*pi/4){
        del_x = -(cur_goal_y-cur_y);
        del_y = cur_goal_x-cur_x;
      }
      //down
      if(cur_ang<=-3*pi/4 ||cur_ang >3*pi/4){
        del_x = -(cur_goal_x-cur_x);
        del_y = -(cur_goal_y-cur_y);
      }
      //left
      if(cur_ang>pi/4 &&cur_ang <=3*pi/4){
        del_x = cur_goal_y-cur_y;
        del_y = -(cur_goal_x-cur_x);
      }
      if (fabs(del_ang)>pi){
        if (del_ang >0){
          del_ang = del_ang - 2*pi;
        }else{
          del_ang = del_ang + 2*pi;
        }
        ROS_INFO("del_ang!!!!!!!!!!!!!!!1:%f",del_ang);
      }
       
      ROS_INFO("for current goal");
      ROS_INFO("del_x:%f, del_y:%f, del_z:%f",del_x,del_y,del_ang);
      if(!reached_waypoint){
        ROS_INFO("cur_goal_x:%f, cur_goal_y:%f, cur_goal_z:%f",cur_goal_x,cur_goal_y,cur_goal_ang);
        driveForwardOdom(fabs(del_x));
        if(fabs(del_ang) > 0.7){
          if (del_ang > 0)
          {
            turnOdom(false,1.57);
          }else{
            turnOdom(true,1.57);
          }
        }
        docking=true;
        while(!docked){
          ros::spinOnce();
          rate.sleep();
        }
        if(docked){
          reached_waypoint = true;
          docking = false;
          ROS_INFO("docked!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
        
      }
      if(reached_waypoint == true ){
        cur_goal_index = cur_goal_index+1;
        if(cur_goal_index<path.size()){
          cur_goal_x = path[cur_goal_index].x;
          cur_goal_y = path[cur_goal_index].y;
          cur_goal_ang = path[cur_goal_index].z;
          reached_waypoint = false;
          docked = false;
          ros::spinOnce();
          ROS_INFO("Next goal");
        }else{
          stop();
          ROS_INFO("FINISH!!!!!!!!!!!!!!!!!!!!!!!!!!!11");
          std_msgs::String msg;
          std::stringstream ss;
          ss << "Done";
          msg.data = ss.str();
          state_pub_.publish(msg);
          state_pub_.publish(msg);
          state_pub_.publish(msg);
          ROS_INFO("AAAA");
          running = false;
          go = false;
          reached_final_goal = true;
        } 
      }
    }
  }
  
  
}


void navigation::stop(){
  geometry_msgs::Twist base_cmd;
  //the command will be to go forward at 0.25 m/s
  base_cmd.linear.y = base_cmd.angular.z = 0;
  base_cmd.linear.x = 0;
  cmd_vel_pub_.publish(base_cmd);
  ROS_INFO("stop!!!!!!!!!");

}


//! Drive forward a specified distance based on odometry information
bool navigation::driveForwardOdom(double distance)
{
  //wait for the listener to get the first message
  listener_.waitForTransform("base_link", "odom", 
                             ros::Time(0), ros::Duration(1.0));
  
  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener_.lookupTransform("base_link", "odom", 
                            ros::Time(0), start_transform);
  
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to go forward at 0.25 m/s
  base_cmd.linear.y = base_cmd.angular.z = 0;
  base_cmd.linear.x = 0.25;
  
  ros::Rate rate(30.0);
  bool done = false;
  while (!done && ros::ok())
  {
    //send the drive command
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    //get the current transform
    try
    {
      listener_.lookupTransform("base_link", "odom", 
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      break;
    }
    //see how far we've traveled
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    double dist_moved = relative_transform.getOrigin().length();

    if(dist_moved > distance) {
      done = true;
      base_cmd.linear.x = 0;
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      ROS_INFO("straight reached");

    }
  }
  if (done) return true;
  return false;
}
bool navigation::turnOdom(bool clockwise, double radians)
{
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;

  //wait for the listener to get the first message
  listener_.waitForTransform("base_link", "odom", 
                             ros::Time(0), ros::Duration(1.0));
  
  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener_.lookupTransform("base_link", "odom", 
                            ros::Time(0), start_transform);
  
  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to turn at 0.75 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.45;
  if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
  
  //the axis we want to be rotating by
  tf::Vector3 desired_turn_axis(0,0,1);
  if (!clockwise) desired_turn_axis = -desired_turn_axis;
  
  ros::Rate rate(10.0);
  bool done = false;
  while (!done && ros::ok())
  {
    //send the drive command
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    //get the current transform
    try
    {
      listener_.lookupTransform("base_link", "odom", 
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      break;
    }
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    tf::Vector3 actual_turn_axis = 
      relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if ( fabs(angle_turned) < 1.0e-2) continue;

    if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians){
      done = true;
      base_cmd.angular.z = 0;
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      ROS_INFO("turning reached");
    }
  }
  if (done) return true;
  return false;
}



int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");

  navigation server;
  server.init();
  ros::Rate rate(20.0);
  
  while(ros::ok()){
    server.controller();
    //ROS_INFO("aaaaaa");
    ros::spinOnce();
    rate.sleep();
  }
}