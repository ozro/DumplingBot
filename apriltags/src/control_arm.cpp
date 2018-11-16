#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <robot_mechanism_controllers/JTCartesianControllerState.h>
#include <apriltags/AprilTagDetections.h>

class VisualServoing
{
  private:
  geometry_msgs::PoseStamped cmd;

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;
  ros::Subscriber cur_pos_;
  ros::Subscriber tag_detection_;

  float target_x ;
  float target_y ;
  float target_z ;
  Eigen::Quaterniond target_q;
  float threshod ;
  bool is_target ;

  public:
  void init()
  {
    cmd.header.frame_id = "/torso_lift_link";
    threshod = 0.01;
    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("r_cart/command_pose", 1);
    cur_pos_ = n_.subscribe("r_cart/state",100,&VisualServoing::cur_pos_callback,this);
    tag_detection_ = n_.subscribe("apriltags/detections",100,&VisualServoing::detection_callback,this);
    ros::NodeHandle n_private("~");
  }

  ~VisualServoing()   { }
  void detection_callback(const apriltags::AprilTagDetections::ConstPtr& message){
    if (message->detections.size()>0){
      this->is_target = true;
      ROS_INFO("x: %f", message->detections[0].pose.position.x);
      ROS_INFO("y: %f", message->detections[0].pose.position.y);
      ROS_INFO("z: %f", message->detections[0].pose.position.z);
      this->target_x = message->detections[0].pose.position.x;
      this->target_y = message->detections[0].pose.position.y;
      this->target_z = message->detections[0].pose.position.z;
      this->target_q.x() = message->detections[0].pose.orientation.x;
      this->target_q.y() = message->detections[0].pose.orientation.y;
      this->target_q.z() = message->detections[0].pose.orientation.z;
      this->target_q.w() = message->detections[0].pose.orientation.w;

    }
    
  }
  void cur_pos_callback(const robot_mechanism_controllers::JTCartesianControllerState::ConstPtr& message){
  	
    if(this->is_target == true){
      Eigen::Quaterniond cur_q;
      cur_q.x() = message->x.pose.orientation.x;
      cur_q.y() = message->x.pose.orientation.y;
      cur_q.z() = message->x.pose.orientation.z;
      cur_q.w() = message->x.pose.orientation.w;
      Eigen::Vector3d cur_euler = cur_q.toRotationMatrix().eulerAngles(0, 1, 2);
      Eigen::Vector3d target_euler = target_q.toRotationMatrix().eulerAngles(0, 1, 2);
      Eigen::Vector3d diff_euler = target_euler- cur_euler;
      Eigen::Vector3d command_euler = cur_euler;
      float angle_step = diff_euler.norm()/20;
      if(fabs(diff_euler[0]) > 0.03){
        command_euler[0] += diff_euler[0]*angle_step;
      }
      if(fabs(diff_euler[1]) > 0.03){
        command_euler[1] += diff_euler[1]*angle_step;
      }
      if(fabs(diff_euler[2]) > 0.03){
        command_euler[2] += diff_euler[2]*angle_step;
      }

      Eigen::Quaternionf command_q;
      command_q = Eigen::AngleAxisf(command_euler[0], Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(command_euler[1], Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(command_euler[2],Eigen::Vector3f::UnitZ());

      cmd.pose.orientation.x =  command_q.x();
      cmd.pose.orientation.y =  command_q.y();
      cmd.pose.orientation.z =  command_q.z();
      cmd.pose.orientation.w =  command_q.w();

      float del_x = -message->x.pose.position.x+this->target_x;
      float del_y = -message->x.pose.position.y+this->target_y;
      float del_z = -message->x.pose.position.z+this->target_z;
      float total_mag = fabs(del_x)+fabs(del_y)+fabs(del_z);
      float step_magnitude= total_mag/20;
      if(fabs(del_x) > 0.03){
        cmd.pose.position.x = message->x.pose.position.x+del_x/total_mag*step_magnitude;
      }else{
        cmd.pose.position.x = message->x.pose.position.x;
      }
      if(fabs(del_y) > 0.03){
        cmd.pose.position.y = message->x.pose.position.y+del_y/total_mag*step_magnitude;
      }else{
        cmd.pose.position.y = message->x.pose.position.y;
      }
      if(fabs(del_z) > 0.03){
        cmd.pose.position.z = message->x.pose.position.z+del_z/total_mag*step_magnitude;
      }else{
        cmd.pose.position.z = message->x.pose.position.z;
      }
      
      
      
      
      //ROS_INFO("currant pose: %f,%f,%f",message->x.pose.position.x,message->x.pose.position.y,message->x.pose.position.z);
      //ROS_INFO("del pose: %f,%f,%f",del_x,del_y,del_z);
      //ROS_INFO("command: %f,%f,%f",cmd.pose.position.x,cmd.pose.position.y,cmd.pose.position.z);
      pose_pub_.publish(cmd);
      this->is_target = false;
    }
  	else{
      ROS_INFO("No target pose");
    }

  };
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_arms_keyboard");
  VisualServoing server;
  server.init();
  while(ros::ok()){
  	ros::spinOnce();
  }
  return(0);
}
