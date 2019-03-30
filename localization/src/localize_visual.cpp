#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h> 



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Publisher odom_pub = 
    node.advertise<nav_msgs::Odometry>("odom", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("/base_link","/map",ros::Time::now(),ros::Duration(4.0));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    nav_msgs::Odometry cur_pose;
    geometry_msgs::PoseStamped vec;
    geometry_msgs::PoseStamped vec_out;
    vec.header.frame_id = "base_link";

    vec.pose.position.x =0;
    vec.pose.position.y = 0;
    vec.pose.position.z = 0;
    vec.pose.orientation.x = 0;
    vec.pose.orientation.y = 0;
    vec.pose.orientation.z = 0;
    vec.pose.orientation.w = 1;
    listener.transformPose("map", vec, vec_out);
    cur_pose.pose.pose = vec_out.pose;
    odom_pub.publish(cur_pose);

    rate.sleep();
  }
  return 0;
};