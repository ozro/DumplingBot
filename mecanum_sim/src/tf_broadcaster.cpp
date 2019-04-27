#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(1000);

  tf::TransformBroadcaster broadcaster;
  
  double pi = 3.1415926535897932385;

  while(n.ok())
  {
   
    /*broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(pi/2,0.0,-pi/2)), tf::Vector3(2.93, 0.0, 0.645)),
        ros::Time::now(),"map", "tag0"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(pi/2,0.0,-pi/2)), tf::Vector3(2.93, 0.61, 0.645)),
        ros::Time::now(),"map", "tag4"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(pi/2,0.0,0.0)), tf::Vector3(1.219, 3.09, 0.645)),
        ros::Time::now(),"map", "tag2"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(pi/2,0.0,-pi/2)), tf::Vector3(2.93, 2.36, 0.645)),
        ros::Time::now(),"map", "tag3"));


    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0, 0.0)), tf::Vector3(0, 0, 0.645)),
        ros::Time::now(),"base_link_visual", "usb_cam"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,pi/2, 0.0)), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"usb_cam", "optical_fake"));
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0, pi/2)), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"optical_fake", "optical"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0, -pi/2)), tf::Vector3(-0.225, -0.21, 0.645)),
        ros::Time::now(),"base_link_visual", "usb_cam1"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,pi/2, 0.0)), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"usb_cam1", "optical_fake1"));
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0, pi/2)), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"optical_fake1", "optical1"));*/
     broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0,pi/2, 0)), tf::Vector3(0, 0, 0.0)),
        ros::Time::now(),"base_link_visual1", "optical"));
     broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0,0.0, -pi/2)), tf::Vector3(0, 0, 0.0)),
        ros::Time::now(),"optical", "usb_cam"));
     broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0,0, -pi/2)), tf::Vector3(-0.225, -0.21, 0.0)),
        ros::Time::now(),"base_link_visual2", "optical2"));
     broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0,0.0, -pi/2)), tf::Vector3(0, 0, 0.0)),
        ros::Time::now(),"optical2", "usb_cam2"));
        
    r.sleep();
  }
}
