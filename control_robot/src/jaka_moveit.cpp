#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h> 
#include<moveit/planning_scene_monitor/planning_scene_monitor.h>
#include<moveit/planning_scene/planning_scene.h>
#include<std_msgs/Float64MultiArray.h>
using namespace std;
geometry_msgs::PoseStamped target_pose;
 geometry_msgs::PoseStamped target_pose_1;

 void handposeCallback(const geometry_msgs::PoseStamped& state_msg)
 {
    target_pose_1.pose.position.x=state_msg.pose.position.x;
    target_pose_1.pose.position.y=state_msg.pose.position.y;
    target_pose_1.pose.position.z=state_msg.pose.position.z;
    std::cout<<"state_msg.pose.position.x"<<state_msg.pose.position.x<<std::endl;
    // std::cout<<"state_msg.pose.position.x"<<state_msg.pose.position.x<<std::endl;
    // std::cout<<"state_msg.pose.position.y"<<state_msg.pose.position.y<<std::endl;
    // std::cout<<"state_msg.pose.position.z"<<state_msg.pose.position.z<<std::endl;
    // Orientation
// std::cout<<"target_pose.pose.position.x: "<<target_pose.pose.position.x<<std::endl;
    target_pose_1.pose.orientation.x=state_msg.pose.orientation.x;
    target_pose_1.pose.orientation.y=state_msg.pose.orientation.y;
    target_pose_1.pose.orientation.z=state_msg.pose.orientation.z;
    target_pose_1.pose.orientation.w=state_msg.pose.orientation.w;
     std::cout<<"target_pose_1.pose.orientation.x"<<target_pose_1.pose.orientation.x<<std::endl;
 }



int main(int argc, char** argv)
{
    ros::init(argc, argv ,"moveit_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_values_publisher=n.advertise<std_msgs::Float64MultiArray>("/joint_values",1);

    ros::Subscriber hand_pose_subscriber =n.subscribe("/phantom/end_effector_pose",1,handposeCallback);

    // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // planning_scene_monitor->startStateMonitor();
    // planning_scene_monitor->startSceneMonitor();
    // planning_scene_monitor->startWorldGeometryMonitor();
ros::AsyncSpinner spinner(1);
spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    std::string end_link=move_group.getEndEffectorLink();
    std::string reference_frame="dummy";
    move_group.setPoseReferenceFrame(reference_frame);
    

    move_group.setPlannerId("EST");


    while(ros::ok())
    {


geometry_msgs::PoseStamped current_pose =move_group.getCurrentPose(end_link);

    target_pose.pose.position.x=current_pose.pose.position.x+0.02*target_pose_1.pose.position.x;
    target_pose.pose.position.y=current_pose.pose.position.y+0.0001*target_pose_1.pose.position.y;
    target_pose.pose.position.z=current_pose.pose.position.z;
    target_pose.pose.orientation.x=current_pose.pose.orientation.x;
    target_pose.pose.orientation.y=current_pose.pose.orientation.y;
    target_pose.pose.orientation.z=current_pose.pose.orientation.z;
    target_pose.pose.orientation.w=current_pose.pose.orientation.w;

        move_group.setPoseTarget(target_pose.pose);
     
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      
      moveit::planning_interface::MoveItErrorCode success=move_group.plan(my_plan);
      ROS_INFO("Plan (pose goal)%s",success?"":"FAIED");
        if(success)
        {

move_group.execute(my_plan);
        }

    }
    ros::shutdown();
    return 0;
}




 






