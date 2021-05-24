#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/TwistStamped.h>
//#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <limits>
#include <ros/subscriber.h>
#include <ros/publisher.h>
//#include <hector_quadrotor_interface/limiters.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <cmath>
#include <pluginlib/class_list_macros.h>
//#include <hector_quadrotor_interface/helpers.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>




//Globale Objekte
geometry_msgs::Twist msg_twist;
geometry_msgs::PoseStamped msg_pose;


//Funktion schreibt mit Hilfe von "call  by reference" auf das globale Objekt 
void BuiltTwistMsg(float p_lx, float p_ly, float p_lz, float p_ax, float p_ay, float p_az, geometry_msgs::Twist& p_msg)
{
	p_msg.linear.x = p_lx;
	p_msg.linear.y = p_ly;
	p_msg.linear.y = p_lz;
		
	p_msg.angular.x = p_ax;
	p_msg.angular.y = p_ay;
	p_msg.angular.z = p_az;
}


//Vorlaufig, später mit Parameterübergabe
void BuiltPoseMsg()
{
	msg_pose.header.frame_id = "world";
	msg_pose.header.seq= 0;
	msg_pose.header.stamp = ros::Time::now();

	msg_pose.pose.position.x = 0.0;
	msg_pose.pose.position.y = 0.0;
	msg_pose.pose.position.z = 0.8;
	
	msg_pose.pose.orientation.x = 0.0;
	msg_pose.pose.orientation.y = 0.0;
	msg_pose.pose.orientation.z = 0.0;
	msg_pose.pose.orientation.w = 0.0;
}

void PositionCallback(const sensor_msgs::LaserScan::ConstPtr& p_msg)
{
	
	//ROS_INFO("Range: %f ", p_msg->ranges[0]);
	int size = sizeof(p_msg->ranges)/sizeof(p_msg->ranges[0]);
	float min = 0;
	for(int i = 0; i < 1081;i++)
	{
	 if(isfinite(p_msg->ranges[i]) && isnormal(p_msg->ranges[i]))
	 {
	 	if ( p_msg->ranges[min] > p_msg->ranges[i] ) 
	 	{
	 		min = i;
	 	}
	 }
	}
	ROS_INFO("Minimum value: %f",p_msg->ranges[min]);
	ROS_INFO("Minimum position: %f",min);
	if(p_msg->ranges[min] < 2)
	{
		BuiltTwistMsg(0.3, 0.0, 0.0, 0.0, 0.0, 2, msg_twist);
	}
	
	else
	{		
		BuiltTwistMsg(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, msg_twist);
	}
	
}


int main ( int argc , char **argv )
{	
	ros:: init( argc , argv , "dang_hector") ;
	ros::NodeHandle n;
	
	//Subscriber und Publisher erstellen
	ros::Subscriber sub_scan = n.subscribe("scan", 1000, PositionCallback);
	ros::Publisher pub_cmdvel = n.advertise <geometry_msgs::Twist>("cmd_vel" , 1000 ) ;
	ros::Publisher pub_pose = n.advertise <geometry_msgs::PoseStamped>("command/pose" , 1000 ) ;
	
	
	ros::Rate loop_rate(1); 	//1 HZ
	
	ROS_INFO("Automatic: Started!");
	
	//einmaliges senden der angestrebeten Flughoehe
	BuiltPoseMsg();
	pub_pose.publish(msg_pose);
	loop_rate.sleep();
	ROS_INFO("Automatic: Flight altitude set!");
	
	
	
	while(ros::ok()) //are you okay?
	{
		
		pub_cmdvel.publish(msg_twist);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	ros::spin();
	
	return 0;
	
}

