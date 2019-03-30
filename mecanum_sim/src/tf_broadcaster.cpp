#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  
  double pi = 3.1415926535897932385;

  while(n.ok())
  {
   
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0,-pi/2)), tf::Vector3(3.5, 0.5, 0.0)),
        ros::Time::now(),"map", "tag13"));
    /*
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0,0.0)), tf::Vector3(-3.5, -0.5, 0.0)),
        ros::Time::now(),"map", "tag9"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0,pi/2)), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"map", "tag12"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0,0.0)), tf::Vector3(-3.5, -0.5, 0.0)),
        ros::Time::now(),"map", "tag11"));
*/


    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,3*pi/4, 0.0)), tf::Vector3(0, 0, 0.5)),
        ros::Time::now(),"base_link", "usb_cam"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0, -pi/2)), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"usb_cam", "optical"));
    
        
    r.sleep();
  }
}
