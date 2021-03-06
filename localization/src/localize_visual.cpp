#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h> 



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Publisher odom_pub = 
    node.advertise<nav_msgs::Odometry>("vo_odom1", 1);

  tf::TransformListener listener;
  listener.waitForTransform("/base_link_visual","/map",ros::Time::now(),ros::Duration(1.0));
  ros::Rate rate(100.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
     // listener.waitForTransform("/base_link","/map",ros::Time::now(),ros::Duration(5.0));
      nav_msgs::Odometry cur_pose;
      geometry_msgs::PoseStamped vec;
      geometry_msgs::PoseStamped vec_out;
      vec.header.frame_id = "base_link_visual";
     // 
      vec.pose.position.x =0;
      vec.pose.position.y = 0;
      vec.pose.position.z = 0;
      vec.pose.orientation.x = 0;
      vec.pose.orientation.y = 0;
      vec.pose.orientation.z = 0;
      vec.pose.orientation.w = 1;
      cur_pose.pose.covariance[0]  = 0.2;
      cur_pose.pose.covariance[7]  = 0.9;
      cur_pose.pose.covariance[14] = 99999;
      cur_pose.pose.covariance[21] = 99999;
      cur_pose.pose.covariance[28] = 99999;
      cur_pose.pose.covariance[35] = 0.7;
      listener.transformPose("map", vec, vec_out);
      cur_pose.header.frame_id = "map";
      cur_pose.child_frame_id = "base_link";
      cur_pose.pose.pose = vec_out.pose;
      cur_pose.header.stamp = ros::Time::now();
      odom_pub.publish(cur_pose);

      rate.sleep();
    }
    catch (...){
      ROS_INFO("aaaaaaaaaaaaaaaa");
      // change to pose from last time
      nav_msgs::Odometry cur_pose;
      //odom_pub.publish(cur_pose);
    }
    
  }
  return 0;
};