#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h> 



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Publisher odom_pub = 
    node.advertise<nav_msgs::Odometry>("vo_odom1", 5);
  ros::Publisher odom2_pub = 
    node.advertise<nav_msgs::Odometry>("vo_odom2", 5);


  tf::TransformListener listener;
  listener.waitForTransform("/base_link_visual1","/map",ros::Time::now(),ros::Duration(1.0));
  listener.waitForTransform("/base_link_visual2","/map",ros::Time::now(),ros::Duration(1.0));
  ros::Rate rate(100.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
     // listener.waitForTransform("/base_link","/map",ros::Time::now(),ros::Duration(5.0));
      nav_msgs::Odometry cur_pose;
      geometry_msgs::PoseStamped vec;
      geometry_msgs::PoseStamped vec_out;
      vec.header.frame_id = "base_link_visual1";
     // 
      vec.pose.position.x =0;
      vec.pose.position.y = 0;
      vec.pose.position.z = 0;
      vec.pose.orientation.x = 0;
      vec.pose.orientation.y = 0;
      vec.pose.orientation.z = 0;
      vec.pose.orientation.w = 1;
      cur_pose.pose.covariance[0]  = 0.04;
      cur_pose.pose.covariance[7]  = 0.16;
      cur_pose.pose.covariance[14] = 99999;
      cur_pose.pose.covariance[21] = 99999;
      cur_pose.pose.covariance[28] = 99999;
      cur_pose.pose.covariance[35] = 0.09;
      listener.transformPose("map", vec, vec_out);
      cur_pose.header.frame_id = "map";
      cur_pose.child_frame_id = "base_link";
      cur_pose.pose.pose = vec_out.pose;
      cur_pose.header.stamp = ros::Time::now();
      odom_pub.publish(cur_pose);

      nav_msgs::Odometry cur_pose1;
      geometry_msgs::PoseStamped vec1;
      geometry_msgs::PoseStamped vec_out1;
      vec1.header.frame_id = "base_link_visual2";
     // 
      vec1.pose.position.x =0;
      vec1.pose.position.y = 0;
      vec1.pose.position.z = 0;
      vec1.pose.orientation.x = 0;
      vec1.pose.orientation.y = 0;
      vec1.pose.orientation.z = 0;
      vec1.pose.orientation.w = 1;
      cur_pose1.pose.covariance[0]  = 0.04;
      cur_pose1.pose.covariance[7]  = 0.16;
      cur_pose1.pose.covariance[14] = 99999;
      cur_pose1.pose.covariance[21] = 99999;
      cur_pose1.pose.covariance[28] = 99999;
      cur_pose1.pose.covariance[35] = 0.09;
      listener.transformPose("map", vec1, vec_out1);
      cur_pose1.header.frame_id = "map";
      cur_pose1.child_frame_id = "base_link";
      cur_pose1.pose.pose = vec_out1.pose;
      cur_pose1.header.stamp = ros::Time::now();
      odom2_pub.publish(cur_pose1);

      rate.sleep();
    }
    catch (...){
      // change to pose from last time
      nav_msgs::Odometry cur_pose;
      //odom_pub.publish(cur_pose);
    }
    
  }
  return 0;
};