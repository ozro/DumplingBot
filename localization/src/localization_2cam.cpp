#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h> 
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Vector3.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <localization/tag_pose_optical.h> 
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
double pi = 3.1415926535897932385;
using namespace message_filters;

typedef sync_policies::ApproximateTime<apriltags::AprilTagDetections, apriltags::AprilTagDetections> MySyncPolicy;

tf::TransformListener* listener;
std::vector<int> Find_biggest_tag(const apriltags::AprilTagDetections::ConstPtr& message,
  const apriltags::AprilTagDetections::ConstPtr& message1){
    int biggest = 0;
    std::vector<int> result;
    int cam_num;
    int index;
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
            index = i;
            cam_num = 0;
        }
    }
    for(int i = 0; i <message1->detections.size(); i ++){
        x1 = message1->detections[i].corners2d[0].x;
        y1 = message1->detections[i].corners2d[0].y;
        x2 = message1->detections[i].corners2d[1].x;
        y2 = message1->detections[i].corners2d[1].y;
        x3 = message1->detections[i].corners2d[2].x;
        y3 = message1->detections[i].corners2d[2].y;
        x4 = message1->detections[i].corners2d[3].x;
        y4 = message1->detections[i].corners2d[3].y;
        area = abs((x1*y2-y1*x2+x2*y3-y2*x3+x3*y4-y3*x4+x4*y1-y4*x1)/2);
        if(area > biggest){
            biggest = area;
            index = i;
            cam_num = 1;
        }
    }
    result.push_back(index);
    result.push_back(cam_num);
    return result;

}



void callback(const apriltags::AprilTagDetections::ConstPtr& message, 
        const apriltags::AprilTagDetections::ConstPtr& message1,ros::Publisher at_pub)
{ 
  

  if(message->detections.size()>0 || message1->detections.size()>0){
    //int id = message->detections[0].id;
    std::vector<int> biggest_info;
    biggest_info = Find_biggest_tag(message,message1);
    int id;
    int index = biggest_info[0];
    int cam_num = biggest_info[1];
    
    if ( cam_num == 0){
      id = message->detections[index].id;
    }else{
      id = message1->detections[index].id;
    }
    //ROS_INFO("index,%d,cam_num,%d, id,%d",index,cam_num,id);
    //ROS_INFO("biggest index %d", index);
    //ROS_INFO("biggest id %d", id);
    float cur_px;
    float cur_py;
    float cur_pz;
    Eigen::Quaterniond cur_q;
    geometry_msgs::PoseStamped vec;
    geometry_msgs::PoseStamped vec_out;
    ros::Time apriltags_time;
    if(cam_num == 0){
      cur_px = message->detections[index].pose.position.x;
      cur_py = message->detections[index].pose.position.y;
      cur_pz = message->detections[index].pose.position.z;
      cur_q.x() = message->detections[index].pose.orientation.x;
      cur_q.y() = message->detections[index].pose.orientation.y;
      cur_q.z() = message->detections[index].pose.orientation.z;
      cur_q.w() = message->detections[index].pose.orientation.w;
      apriltags_time = message->header.stamp;
      vec.header.frame_id = "optical";
      vec.header.stamp = apriltags_time;
      vec.pose.position.x =cur_px;
      vec.pose.position.y = cur_py;
      vec.pose.position.z = cur_pz;
      vec.pose.orientation.x = cur_q.x();
      vec.pose.orientation.y = cur_q.y();
      vec.pose.orientation.z = cur_q.z();
      vec.pose.orientation.w = cur_q.w();
    }else{
      cur_px = message1->detections[index].pose.position.x;
      cur_py = message1->detections[index].pose.position.y;
      cur_pz = message1->detections[index].pose.position.z;
      cur_q.x() = message1->detections[index].pose.orientation.x;
      cur_q.y() = message1->detections[index].pose.orientation.y;
      cur_q.z() = message1->detections[index].pose.orientation.z;
      cur_q.w() = message1->detections[index].pose.orientation.w;
      apriltags_time = message1->header.stamp;
      vec.header.frame_id = "optical1";
      vec.header.stamp = apriltags_time;
      vec.pose.position.x =cur_px;
      vec.pose.position.y = cur_py;
      vec.pose.position.z = cur_pz;
      vec.pose.orientation.x = cur_q.x();
      vec.pose.orientation.y = cur_q.y();
      vec.pose.orientation.z = cur_q.z();
      vec.pose.orientation.w = cur_q.w();
    }
    localization::tag_pose_optical cur_tag;
    cur_tag.header = vec.header;
    cur_tag.tag_pose = vec;
    cur_tag.tag_id = id;
    at_pub.publish(cur_tag);
    /*
    //listener->setExtrapolationLimit(ros::Duration(10.0));
    listener->transformPose("base_link", vec, vec_out);
    
    //ROS_INFO("tf: %f, %f, %f", vec_out.pose.position.x,vec_out.pose.position.y,vec_out.pose.position.z);
    float ox = vec_out.pose.orientation.x;
    float oy = vec_out.pose.orientation.y;
    float oz = vec_out.pose.orientation.z;
    float ow = vec_out.pose.orientation.w;
    tf::Quaternion quat(ox,oy,oz,ow);
    tf::TransformBroadcaster broadcaster;
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //ROS_INFO("tf: %f", yaw);
    tf::Transform tag_in_base = tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(roll,pitch,yaw)), tf::Vector3(vec_out.pose.position.x, vec_out.pose.position.y, vec_out.pose.position.z));
    tf::StampedTransform base_in_tag(tag_in_base.inverse(),ros::Time::now(),tag_name,"base_link");
    broadcaster.sendTransform(tf::StampedTransform(base_in_tag,ros::Time::now(),tag_name,"base_link"));*/    
  }
}
int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
    ros::init(argc, argv,"localization");
    ros::NodeHandle nh;
    listener = new (tf::TransformListener);
    listener->waitForTransform("/base_link_visual","/optical",ros::Time::now(),ros::Duration(3.0));
    tf::TransformBroadcaster broadcaster;
    ros::Publisher at_pub = nh.advertise<localization::tag_pose_optical>("pose_from_at", 10);
    message_filters::Subscriber<apriltags::AprilTagDetections> detection_sub(nh, "/apriltags/detections", 1);
    message_filters::Subscriber<apriltags::AprilTagDetections> detection1_sub(nh, "/apriltags/detections1", 1);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), detection_sub, detection1_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2,at_pub));
   
    while(ros::ok()){
      //server.localize();
      ros::spinOnce();
      //ros::Duration(0.01).sleep();

    }
    return 0;
}
