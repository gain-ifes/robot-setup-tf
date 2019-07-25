#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

geometry_msgs::Pose pose_msg;
geometry_msgs::Twist vel_msg;

void VelocityCallback(const geometry_msgs::Twist & msg){
	vel_msg = msg;
}

void PoseCallback(const geometry_msgs::Pose & msg){
	pose_msg = msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tf_odom_node");

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	ros::Subscriber subVel= n.subscribe("/Vel",10,&VelocityCallback);
	ros::Subscriber subPos= n.subscribe("/Postura",10,&PoseCallback);

	tf::TransformBroadcaster odom_broadcaster;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(50);
	while(n.ok()){

		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = pose_msg.position.x;
		odom_trans.transform.translation.y = pose_msg.position.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = pose_msg.orientation;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = pose_msg.position.x;
		odom.pose.pose.position.y = pose_msg.position.y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = pose_msg.orientation;

		//set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = vel_msg.linear.x;
		odom.twist.twist.linear.y = vel_msg.linear.y;
		odom.twist.twist.angular.z = vel_msg.angular.z;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;
		r.sleep();
  }
}
