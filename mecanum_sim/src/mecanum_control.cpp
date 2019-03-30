#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <stdlib.h> 

class Mecanum_drive{
	private:
    ros::NodeHandle n_;
    ros::Subscriber cmd_vel_;
    ros::Publisher command_pub;
    float base_width;
    float base_length;
    float wheel_gap;
    float wheel_setback;
    float wheel_radius;
    
    
  public:
    
    void init(){
      cmd_vel_ = n_.subscribe("/cmd_vel",100,&Mecanum_drive::vel_callback,this);
      command_pub = n_.advertise<std_msgs::Float32MultiArray>("mecanum_cmd",10);
      ros::NodeHandle n_private("~");
      base_width = 0.5;
      base_length = 0.5;
      wheel_gap = 0.0508;
      wheel_setback = 0.041275;
      wheel_radius = 0.1016;      
    }
    ~Mecanum_drive(){}
    void vel_callback(const geometry_msgs::Twist::ConstPtr& velocity);
};

void Mecanum_drive::vel_callback(const geometry_msgs::Twist::ConstPtr& velocity){
  float DISTANCE_LEFT_TO_RIGHT_WHEEL = this->base_width +2*this->wheel_gap;
  float DISTANCE_FRONT_TO_REAR_WHEEL = this->base_length -2*this->wheel_setback;
  float WHEEL_SEPARATION_WIDTH = DISTANCE_LEFT_TO_RIGHT_WHEEL / 2;
  float WHEEL_SEPARATION_LENGTH = DISTANCE_FRONT_TO_REAR_WHEEL / 2;
  float x = velocity->linear.x;
  float y = velocity->linear.y;
  float theta = velocity->angular.z;
  float wheel_front_left = (1/this->wheel_radius) * (x - y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*theta);
  float wheel_front_right = (1/this->wheel_radius) * (x + y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*theta);
  float wheel_rear_left = (1/this->wheel_radius) * (x + y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*theta);
  float wheel_rear_right = (1/this->wheel_radius) * (x - y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*theta);
  
  std_msgs::Float32MultiArray cmd;
  // set up dimensions
  cmd.data.push_back(wheel_front_left);
  cmd.data.push_back(wheel_front_right);
  cmd.data.push_back(wheel_rear_left);
  cmd.data.push_back(wheel_rear_right);
  ROS_INFO("front_left:%f",wheel_front_left);
  ROS_INFO("front_right:%f",wheel_front_right);
  ROS_INFO("rear_left:%f",wheel_rear_left);
  ROS_INFO("rear_right:%f",wheel_rear_right);
  command_pub.publish(cmd);
  
}



int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
    ros::init(argc, argv,"Mecanum_drive");
    Mecanum_drive send_command;
    send_command.init();
    while(ros::ok()){
      ros::spinOnce();
      

    }
    return 0;
}

