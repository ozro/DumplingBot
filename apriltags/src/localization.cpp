#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
//#include <Eigen/Geometry>
#include <apriltags/AprilTagDetections.h>

class Localization
{
  private:
  ros::NodeHandle n_;
  ros::Publisher fr_pub_;
  ros::Publisher fl_pub_;
  ros::Publisher br_pub_;
  ros::Publisher bl_pub_;
  ros::Subscriber tag_detection_;

  public:
  void init()
  {
    fr_pub_ = n_.advertise<std_msgs::Float32>("front_right", 1);
    fl_pub_ = n_.advertise<std_msgs::Float32>("front_left", 1);
    br_pub_ = n_.advertise<std_msgs::Float32>("back_right", 1);
    bl_pub_ = n_.advertise<std_msgs::Float32>("back_left", 1);
    tag_detection_ = n_.subscribe("apriltags/detections",100,&Localization::detection_callback,this);
    ros::NodeHandle n_private("~");
  }

  ~Localization()   { }
  void detection_callback(const apriltags::AprilTagDetections::ConstPtr& message){
    if (message->detections.size()>0){
      float cur_x = message->detections[0].pose.position.x;
      float cur_y = message->detections[0].pose.position.y;
      float cur_z = message->detections[0].pose.position.z;
      std_msgs::Float32 fl;
      fl.data = -1.0f;
      std_msgs::Float32 fr;
      fr.data = 1.0f;
      std_msgs::Float32 bl;
      bl.data = 1.0f;
      std_msgs::Float32 br;
      br.data = -1.0f;
      this->fl_pub_.publish(fl);
      this->fr_pub_.publish(fr);
      this->bl_pub_.publish(bl);
      this->br_pub_.publish(br);
      /*br_pub_.publish(-1.0);
      while(abs(cur_x)>0.05){
        if(cur_x >0){
          fl_pub_.publish(-1.0);
          fr_pub_.publish(1.0);
          bl_pub_.publish(1.0);
          br_pub_.publish(-1.0);
        }
      }*/
      ROS_INFO("x: %f", message->detections[0].pose.position.x);
      ROS_INFO("y: %f", message->detections[0].pose.position.y);
      ROS_INFO("z: %f", message->detections[0].pose.position.z);

    }
    
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "localize");
  Localization server;
  server.init();
  while(ros::ok()){
  	ros::spinOnce();
  }
  return(0);
}