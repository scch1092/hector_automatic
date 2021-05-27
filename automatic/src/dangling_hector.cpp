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
typedef enum {straight, left, right}direction;


//Funktion schreibt mit Hilfe von "call  by reference" auf das globale Objekt 
void BuiltTwistMsg(float p_lx, float p_ly, float p_lz, float p_ax, float p_ay, float p_az, geometry_msgs::Twist& p_msg)
{
	p_msg.linear.x = p_lx;
	p_msg.linear.y = p_ly;
	p_msg.linear.z = p_lz;
		
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
	float min = 540;
	for(int i = 360; i < 720;i++) //standard kegel
	{
	 if(isfinite(p_msg->ranges[i]) && isnormal(p_msg->ranges[i]) )
	 {
	 	if ( p_msg->ranges[min] > p_msg->ranges[i] ) 
	 	{
	 		min = i;
	 	}
	 } 
	}
	float min_rechts = 180;
	for(int i = 359; i > 180;i--) //rechts kegel
	{
	 if(isfinite(p_msg->ranges[i]) && isnormal(p_msg->ranges[i]) )
	 {
	 	if ( p_msg->ranges[min_rechts] > p_msg->ranges[i] ) 
	 	{
	 		min_rechts = i;
	 	}
	 } 
	}
	float min_links = 900;
	for(int i = 721; i < 900;i++) //links kegel
	{
	 if(isfinite(p_msg->ranges[i]) && isnormal(p_msg->ranges[i]) )
	 {
	 	if ( p_msg->ranges[min_links] > p_msg->ranges[i] ) 
	 	{
	 		min_links = i;
	 	}
	 } 
	}
	ROS_INFO("Minimum value: %f",p_msg->ranges[min]);
	static int counter = 0;
	static direction dir = straight;
	static int time_counter = 0;
	ROS_INFO("Counter: %d",counter);
	ROS_INFO("Direction: %d", dir);
	if(counter > 4)
	{	
		ROS_INFO("Ich will drehen!");
		BuiltTwistMsg(0.0, 0.0, 0.0, 0.0, 0.0, 4, msg_twist);
		time_counter++;

		if(time_counter >= 80)
		{
			counter = 0;
			time_counter = 0;
		}
		
	}		
	else
	{
		if(p_msg->ranges[min] < 0.6) //if im vorderen Kegel kleiner 1
			if(p_msg->ranges[360] < p_msg->ranges[720]) //if rechts kleiner als links
			{
				BuiltTwistMsg(0.0, 0.0, 0.0, 0.0, 0.0, -2, msg_twist);
				if(dir == left)
				{
					counter++;
				}
				dir = right;		
			}
			else
			{
				BuiltTwistMsg(0.0, 0.0, 0.0, 0.0, 0.0, 2, msg_twist);
				if(dir == right)
				{
					counter++;
				}
				dir = left;
			}
		else
		{
			
			if(p_msg->ranges[min_links] < 0.4 || p_msg->ranges[min_rechts] < 0.4) 
			{
				if(p_msg->ranges[min_links] > p_msg->ranges[min_rechts])
				{
					BuiltTwistMsg(0.0, 0.0, 0.0, 0.0, 0.0, -1, msg_twist);
					if(dir == left)
					{
						counter++;
					}
					dir = right;
				}
				else
				{
					BuiltTwistMsg(0.0, 0.0, 0.0, 0.0, 0.0, 1, msg_twist);
					if(dir == right)
					{
						counter++;
					}
					dir = left;	
				}
			}
			else
			{
				BuiltTwistMsg(0.25, 0.0, 0.0, 0.0, 0.0, 0.0, msg_twist);
				dir = straight;
				counter = 0;
			}
			
		}
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
	
	
	ros::Rate loop_rate(20); 	//1 HZ
	
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

