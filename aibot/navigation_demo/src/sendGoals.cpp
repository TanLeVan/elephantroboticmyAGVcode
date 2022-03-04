#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>

//使用ROS中actionlib
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation_goals");

	MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}
	
	//定义目标
	move_base_msgs::MoveBaseGoal goal;

	//设置 move_base_msgs/MoveBaseGoal 信息
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 0.14;
	goal.target_pose.pose.position.y = 0.64;
	goal.target_pose.pose.orientation.z = 0.69;
	goal.target_pose.pose.orientation.w = 0.71;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have arrived to the goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	return 0;
}
