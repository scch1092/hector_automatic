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

void PositionCallback(const sensor_msgs::LaserScan::ConstPtr& p_msg)
{
	ROS_INFO("Range: %f, Increment: %f", p_msg->ranges[0], p_msg->angle_increment);
	//ROS_INFO("I got a hit!");
}


int main ( int argc , char **argv )
{	
	ros:: init( argc , argv , "automatic") ;
	ros::NodeHandle n;
	
	ros::Subscriber cont_sub = n.subscribe("scan", 1000, PositionCallback);


	ros::Publisher cont_pub = n.advertise <geometry_msgs::PoseStamped>("command/pose" , 1000 ) ;
	ros::Rate loop_rate(5); 	//1 HZ
	
	ROS_INFO("Automatic start!");
	
	float i = 0.0;
	
	while(ros::ok()) //are you okay?
	{
		geometry_msgs::PoseStamped msg;
		
		msg.header.frame_id = "world";
		msg.header.seq= 0;
        msg.header.stamp = ros::Time::now();

        msg.pose.position.x = i;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.8;
		
		msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 0.0;

		cont_pub.publish(msg);
		
		
		ROS_INFO("Published Automatic");
		
		ros::spinOnce();
		
		loop_rate.sleep();
		i += 0.1;
	}
	
	ros::spin();
	
	return 0;
	
}

