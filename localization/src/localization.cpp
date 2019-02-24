#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h> 
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h> 
#include <fstream>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

class localization{
	private:
    ros::NodeHandle n_;
    ros::Subscriber april_detection;
    ros::Publisher command_pub;
    ros::Publisher odom_pub;
    std::vector<std::vector<float> > map;
    Eigen::Quaterniond cur_q;
    geometry_msgs::Vector3 cur_pose;
    float cur_px;
    float cur_py;
    float cur_pz;
    float del_x;
    float del_y;
    float tag_x;
    float tag_y;
    float dist;
    float update_x;
    float update_y;
    float update_angle;
  public:
    
    void init(){
      april_detection = n_.subscribe("/apriltags/detections",100,&localization::detection_callback,this);
      command_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel",10);
      odom_pub = n_.advertise<geometry_msgs::Vector3>("odom",10);
      ros::NodeHandle n_private("~");
      initialize_map();

    }
    ~localization(){}
    void send_vel(geometry_msgs::Twist cmd,int t);
    void initialize_map();
    void detection_callback(const apriltags::AprilTagDetections::ConstPtr& detections);
};
void localization::initialize_map(){
  std::ifstream infile("/home/jessyxie/dumpling_ws/src/localization/include/localization/map.csv");
  while(infile){
    std::string s;
    if (!getline( infile, s )) break;
    std::istringstream ss( s );
    std::vector <float> record;
    while (ss)
    {
      std::string s;
      if (!getline( ss, s, ',' )) break;
        float a = strtof(s.c_str(),NULL);
        record.push_back( a );
    }
    this->map.push_back(record);
  }

}

void localization::detection_callback(const apriltags::AprilTagDetections::ConstPtr& message){
  if(message->detections.size()>0){
    int id = message->detections[0].id;
    float tag_x = this->map[id][1];
    float tag_y = this->map[id][2];

    //ROS_INFO("x:%f",message->detections[0].pose.position.x);
    //ROS_INFO("y:%f",message->detections[0].pose.position.y);
    //ROS_INFO("z:%f",message->detections[0].pose.position.z);
    cur_px = message->detections[0].pose.position.x;
    cur_py = message->detections[0].pose.position.y;
    
    cur_q.x() = message->detections[0].pose.orientation.x;
    cur_q.y() = message->detections[0].pose.orientation.y;
    cur_q.z() = message->detections[0].pose.orientation.z;
    cur_q.w() = message->detections[0].pose.orientation.w;
    Eigen::Vector3d cur_euler = cur_q.toRotationMatrix().eulerAngles(0, 1, 2);
    dist = sqrt((0.5-cur_py)*(0.5-cur_py)+cur_px*cur_px);
    update_angle =cur_euler[2];
    del_x = dist*cos(update_angle); 
    del_y = dist*sin(update_angle);
    update_x = tag_x-del_x;
    update_y = tag_y-del_y;
    
    ROS_INFO("update_x %f",update_x);
    ROS_INFO("update_y %f",update_y);
    
    cur_pose.x =update_x;
    cur_pose.y = update_y;
    cur_pose.z = update_angle;
    odom_pub.publish(cur_pose);
  }
}


int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
    ros::init(argc, argv,"localization");
    localization server;
    server.init();
    while(ros::ok()){
      //server.send_vel();
      ros::spinOnce();
      ros::Duration(0.1).sleep();

    }
    return 0;
}
