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
  
  public:
    
    void init(){
      april_detection = n_.subscribe("/apriltags/detections",5,&localization::detection_callback,this);
      command_pub = n_.advertise<geometry_msgs::Twist>("cmd_vel",10);
      odom_pub = n_.advertise<nav_msgs::Odometry>("vo_odom1",10);
      ros::NodeHandle n_private("~");
      //initialize_map();

    }
    ~localization(){}
    //void send_vel(geometry_msgs::Twist cmd,int t);
    //void initialize_map();
    void detection_callback(const apriltags::AprilTagDetections::ConstPtr& detections);
};
/*
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
*/
int Find_biggest_tag(const apriltags::AprilTagDetections::ConstPtr& message){
    int biggest = 0;
    int return_idex;
    int area;
    int x1,x2,x3,x4;
    int y1,y2,y3,y4;
    for(int i = 0; i <message->detections.size(); i ++){
        x1 = message->detections[i].corners2d[0].x;
        y1 = message->detections[i].corners2d[0].y;
        x2 = message->detections[i].corners2d[1].x;
        y2 = message->detections[i].corners2d[1].y;
        x3 = message->detections[i].corners2d[2].x;
        y3 = message->detections[i].corners2d[2].y;
        x4 = message->detections[i].corners2d[3].x;
        y4 = message->detections[i].corners2d[3].y;
        area = abs((x1*y2-y1*x2+x2*y3-y2*x3+x3*y4-y3*x4+x4*y1-y4*x1)/2);
        if(area > biggest){
            biggest = area;
            return_idex = i;
        }
    }
    return return_idex;

}

void localization::detection_callback(const apriltags::AprilTagDetections::ConstPtr& message){
  if(message->detections.size()>0){
    //int id = message->detections[0].id;
    int index = Find_biggest_tag(message);
    int id = message->detections[index].id;
    std::string tag_name;
    //ROS_INFO("biggest index %d", index);
    //ROS_INFO("biggest id %d", id);
    
    if (id ==0){tag_name = "tag0";}
    else if (id ==1){tag_name = "tag1";}
    else if (id ==2){tag_name = "tag2";}
    else if (id ==3){tag_name = "tag3";}
    else if (id ==4){tag_name = "tag4";}
    else if (id ==5){tag_name = "tag5";}
    else if (id ==6){tag_name = "tag6";}
    else if (id ==7){tag_name = "tag7";}
    else if (id ==8){tag_name = "tag8";}
    else if (id ==9){tag_name = "tag9";}
    else if (id ==10){tag_name = "tag10";}
    else if (id ==11){tag_name = "tag11";}
    else if (id ==12){tag_name = "tag12";}
    else if (id ==13){tag_name = "tag13";}
    else if (id ==14){tag_name = "tag14";}
    else if (id ==15){tag_name = "tag15";}
    else if (id ==16){tag_name = "tag16";}
    else if (id ==17){tag_name = "tag17";}
    else if (id ==18){tag_name = "tag18";}
    else if (id ==19){tag_name = "tag19";}
    else if (id ==20){tag_name = "tag20";}
    else{
        tag_name = "Unknown";
    }

    cur_px = message->detections[index].pose.position.x;
    cur_py = message->detections[index].pose.position.y;
    cur_pz = message->detections[index].pose.position.z;
    cur_q.x() = message->detections[index].pose.orientation.x;
    cur_q.y() = message->detections[index].pose.orientation.y;
    cur_q.z() = message->detections[index].pose.orientation.z;
    cur_q.w() = message->detections[index].pose.orientation.w;
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
    
    listener.transformPose("base_link_visual", vec, vec_out);

    //ROS_INFO("tf: %f, %f, %f", vec_out.pose.position.x,vec_out.pose.position.y,vec_out.pose.position.z);
    float ox = vec_out.pose.orientation.x;
    float oy = vec_out.pose.orientation.y;
    float oz = vec_out.pose.orientation.z;
    float ow = vec_out.pose.orientation.w;
    tf::Quaternion quat(ox,oy,oz,ow);
  
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //ROS_INFO("tf: %f", yaw);
    tf::Transform tag_in_base = tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(roll,pitch,yaw)), tf::Vector3(vec_out.pose.position.x, vec_out.pose.position.y, vec_out.pose.position.z));
    tf::StampedTransform base_in_tag(tag_in_base.inverse(),ros::Time::now(),tag_name,"base_link_visual");
    broadcaster.sendTransform(tf::StampedTransform(base_in_tag,ros::Time::now(),tag_name,"base_link_visual"));    
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
      ros::Duration(0.01).sleep();

    }
    return 0;
}
