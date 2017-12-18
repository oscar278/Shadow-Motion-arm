#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
int main(int argc, char** argv){
	ros::init(argc,argv,"alvartf");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(1);
	int tim=1;
	spinner.start();
	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::DisplayTrajectory display_trajectory;
	tf::TransformListener listener;
	ros::Rate rate (10.0);
	geometry_msgs::Pose target_pose1;
	while (node.ok()){
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/camera1","/ar_marker_6",ros::Time(0),transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		double x=transform.getOrigin().x();
		double y=transform.getOrigin().y();
		double z=transform.getOrigin().z();
		double yaw=tf::getYaw(transform.getRotation());
		ROS_INFO("%f-  %f - %f - %f",x,y,z,yaw);
		rate.sleep();
group.move();
target_pose1.position.x =(-2)*x;
target_pose1.position.y =(-1)*y;
target_pose1.position.z =(1)*z;
std::vector<geometry_msgs::Pose>waypoints;
waypoints.push_back(target_pose1);

moveit_msgs::RobotTrajectory trajectory;
double fraction = group.computeCartesianPath(waypoints,0.01,0.0,trajectory);
group.setPoseTarget(target_pose1);
moveit::planning_interface::MoveGroup::Plan my_plan;
		
	}

	return 0;
};