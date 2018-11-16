#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <apriltags/AprilTagDetections.h>

class TransformToTF{
public:
	ros::NodeHandle nh_;
	ros::Subscriber apriltag_pose_;
	
	TransformToTF(){
		apriltag_pose_ = nh_.subscribe("apriltags/detections",100,&TransformToTF::detection_callback,this);
	 };
	void detection_callback(const apriltags::AprilTagDetections::ConstPtr& message){
	    if (message->detections.size()>0){
	    	static tf::TransformBroadcaster br;
	    	for(int num = 0; num < message->detections.size(); num++){
	    		std::string name = "tag"+patch::to_string(num);
				tf::Transform transform;
				transform.setOrigin( tf::Vector3(message->detections[num].pose.position.x, 
												   message->detections[num].pose.position.y, 
												   message->detections[num].pose.position.z) );
				transform.setRotation( tf::Quaternion(message->detections[num].pose.orientation.x, 
														message->detections[num].pose.orientation.y, 
														message->detections[num].pose.orientation.z, 
														message->detections[num].pose.orientation.w) );
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));

	    	} 
	      
	    }
    
    }

};


int main (int argc, char** argv) {

	ros::init(argc, argv, "TransformToTF", ros::init_options::NoSigintHandler);
	TransformToTF TransformToTF;
	while(ros::ok()){
		ros::spinOnce();
	}
	

	return 0;
}