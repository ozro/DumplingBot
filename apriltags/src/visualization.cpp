#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <math.h>

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
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "usb_cam", name));
	    	}
	    	if(message->detections.size()==4){
	    		float x0;
	    		float y0;
	    		float z0;
	    		float x1;
	    		float y1;
	    		float z1;
	    		float x2;
	    		float y2;
	    		float z2;
	    		float x3;
	    		float y3;
	    		float z3;
	    		for(int num = 0; num < message->detections.size(); num++){
	    			if(message->detections[num].id == 0){
	    				x0 = message->detections[num].pose.position.x;
	    				y0 = message->detections[num].pose.position.y;
	    				z0 = message->detections[num].pose.position.z;
	    			}
	    			if(message->detections[num].id == 1){
	    				x1 = message->detections[num].pose.position.x;
	    				y1 = message->detections[num].pose.position.y;
	    				z1 = message->detections[num].pose.position.z;
	    			}
	    			if(message->detections[num].id == 2){
	    				x2 = message->detections[num].pose.position.x;
	    				y2 = message->detections[num].pose.position.y;
	    				z2 = message->detections[num].pose.position.z;
	    			}
	    			if(message->detections[num].id == 3){
	    				x3 = message->detections[num].pose.position.x;
	    				y3 = message->detections[num].pose.position.y;
	    				z3 = message->detections[num].pose.position.z;
	    			}
	    		}

	    		float width1 = pow(pow(x1-x0,2)+pow(y1-y0,2)+pow(z1-z0,2),0.5);
	    		float width2 = pow(pow(x3-x2,2)+pow(y3-y2,2)+pow(z3-z2,2),0.5);
	    		float height1 = pow(pow(x0-x2,2)+pow(y0-y2,2)+pow(z0-z2,2),0.5);
	    		float height2 = pow(pow(x1-x3,2)+pow(y1-y3,2)+pow(z1-z3,2),0.5);

	    		float avx = (x0+x1+x2+x3)/4;
	    		float avy = (y0+y1+y2+y3)/4;
	    		float avz = (z0+z1+z2+z3)/4;
	    		float avdis = pow((pow(avx,2)+pow(avy,2)+pow(avz,2)),0.5);
	    		//std::cout<<"Width"<<(width1+width2)/2<<std::endl;
	    		//std::cout<<"Height"<<(height1+height2)/2<<std::endl;

	    		ROS_INFO("Width: %f", (width1+width2)/2);
	    		ROS_INFO("Height: %f", (height1+height2)/2);
	    		ROS_INFO("Distance: %f", avdis);

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
