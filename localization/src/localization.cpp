#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h> 
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <fstream>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
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
    ros::Subscriber april_detection;
    ros::Publisher command_pub;
    ros::Publisher odom_pub;
    std::vector<std::vector<float> > map;
    Eigen::Quaterniond cur_q;
    nav_msgs::Odometry cur_pose;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
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
    bool updated;
  public:
    
    void init(){
      april_detection = n_.subscribe("/apriltags/detections",100,&localization::detection_callback,this);
      command_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel",10);
      odom_pub = n_.advertise<nav_msgs::Odometry>("vo_odom",10);
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
    std::string tag_name;
    
    if (id ==5){
      tag_name = "tag5";
    }
    if (id ==13){
      tag_name = "tag13";
    }
    if (id ==9){
      tag_name = "tag9";
    }
    if (id ==12){
      tag_name = "tag12";
    }
    if (id ==11){
      tag_name = "tag11";
    }
    float tag_x = this->map[id][1];
    float tag_y = this->map[id][2];
    float tag_z = this->map[id][3];
    float tag_ox = this->map[id][4];
    float tag_oy = this->map[id][5];
    float tag_oz = this->map[id][6];
    float tag_ow = this->map[id][7];
    ROS_INFO("aaaaaaaaaaaaaaaaaaaaa%i,%f,%f,%f,%f,%f,%f,%f",id,tag_x,tag_y,tag_z,tag_ox,tag_oy,tag_oz,tag_ow);
    /*
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(tag_x,tag_y, tag_z)),
        ros::Time::now(),"map", "test_tag"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(3/4*pi,0.0,0.0)), tf::Vector3(0,0, 0)),
        ros::Time::now(),"test_tag", "test_tag1"));*/
    ROS_INFO("x:%f",message->detections[0].pose.position.x);
    ROS_INFO("y:%f",message->detections[0].pose.position.y);
    ROS_INFO("z:%f",message->detections[0].pose.position.z);
    cur_px = message->detections[0].pose.position.x;
    cur_py = message->detections[0].pose.position.y;
    cur_pz = message->detections[0].pose.position.z;
    cur_q.x() = message->detections[0].pose.orientation.x;
    cur_q.y() = message->detections[0].pose.orientation.y;
    cur_q.z() = message->detections[0].pose.orientation.z;
    cur_q.w() = message->detections[0].pose.orientation.w;
    geometry_msgs::PoseStamped vec;
    geometry_msgs::PoseStamped vec_out;
    vec.header.frame_id = "optical";
    vec.pose.position.x =cur_px;
    vec.pose.position.y = cur_py;
    vec.pose.position.z = cur_pz;
    vec.pose.orientation.x = cur_q.x();
    vec.pose.orientation.y = cur_q.y();
    vec.pose.orientation.z = cur_q.z();
    vec.pose.orientation.w = cur_q.w();
    
    listener.transformPose("base_link", vec, vec_out);

    ROS_INFO("tf: %f, %f, %f", vec_out.pose.position.x,vec_out.pose.position.y,vec_out.pose.position.z);
    float ox = vec_out.pose.orientation.x;
    float oy = vec_out.pose.orientation.y;
    float oz = vec_out.pose.orientation.z;
    float ow = vec_out.pose.orientation.w;
    tf::Quaternion quat(ox,oy,oz,ow);
  
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    tf::Transform tag_in_base = tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(roll,pitch,yaw)), tf::Vector3(vec_out.pose.position.x, vec_out.pose.position.y, vec_out.pose.position.z));
    tf::StampedTransform base_in_tag(tag_in_base.inverse(),ros::Time::now(),tag_name,"base_link");
    broadcaster.sendTransform(tf::StampedTransform(base_in_tag,ros::Time::now(),tag_name,"base_link"));    
  }
}


int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
    ros::init(argc, argv,"localization");
    localization server;
    server.init();
    tf::TransformListener listener;
    while(ros::ok()){
      //server.localize();
      ros::spinOnce();
      ros::Duration(0.1).sleep();

    }
    return 0;
}
