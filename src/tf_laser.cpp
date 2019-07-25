#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "tf_laser_node");
	ros::NodeHandle n;
   
	static tf::TransformBroadcaster laser_broadcaster;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
    
    ros::Rate r(50);

	while(n.ok()){
		// check for incoming messages
		ros::spinOnce();              
		 
		current_time = ros::Time::now();
		
		//laser_broadcaster.sendTransform(
		//tf::StampedTransform(
			//tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 0.0, 0.0)),
			//ros::Time::now(),"base_link", "laser_frame"));
			
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped laser_trans;
		laser_trans.header.stamp = current_time;
		laser_trans.header.frame_id = "base_link";
		laser_trans.child_frame_id = "laser_frame";
		laser_trans.transform.translation.x = -0.13;
		laser_trans.transform.translation.y = 0.0;
		laser_trans.transform.translation.z = 0.0;
		laser_trans.transform.rotation.x = 0.0;
		laser_trans.transform.rotation.y = 0.0;
		laser_trans.transform.rotation.z = 0.0;
		laser_trans.transform.rotation.w = 1.0;

		//send the transform
		laser_broadcaster.sendTransform(laser_trans);
		
		last_time = current_time;
		r.sleep();
	}
}
