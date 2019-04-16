#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "message_filters/subscriber.h"
#include <geometry_msgs/PoseStamped.h> 
#include <localization/tag_pose_optical.h> 

class tffilter
{
public:
  tffilter() : tf_(),  target_frame_("base_link")
  {
    point_sub_.subscribe(n_, "pose_from_at", 10);
    tf_filter_ = new tf::MessageFilter<localization::tag_pose_optical>(point_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&tffilter::msgCallback, this, _1) );
  } ;

private:
  message_filters::Subscriber<localization::tag_pose_optical> point_sub_;
  tf::TransformListener tf_;
  tf::TransformBroadcaster broadcaster;
  tf::MessageFilter<localization::tag_pose_optical> * tf_filter_;
  ros::NodeHandle n_;
  std::string target_frame_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const localization::tag_pose_optical>& point_ptr) 
  {
    geometry_msgs::PoseStamped vec_out;
    geometry_msgs::PoseStamped vec_in;
    vec_in = point_ptr->tag_pose;
    int id = point_ptr->tag_id;
    std::string tag_name;
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
    try 
    {
      tf_.transformPose(target_frame_, vec_in, vec_out);
      float ox = vec_out.pose.orientation.x;
      float oy = vec_out.pose.orientation.y;
      float oz = vec_out.pose.orientation.z;
      float ow = vec_out.pose.orientation.w;
      tf::Quaternion quat(ox,oy,oz,ow);
      
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      //ROS_INFO("tf: %f", yaw);
      tf::Transform tag_in_base = tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(roll,pitch,yaw)), tf::Vector3(vec_out.pose.position.x, vec_out.pose.position.y, vec_out.pose.position.z));
      tf::StampedTransform base_in_tag(tag_in_base.inverse(),ros::Time::now(),tag_name,"base_link");
      broadcaster.sendTransform(tf::StampedTransform(base_in_tag,ros::Time::now(),tag_name,"base_link"));
    }
    catch (tf::TransformException &ex) 
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tf_filter"); //Init ROS
  tffilter filter; //Construct class
  ros::spin(); // Run until interupted 
};