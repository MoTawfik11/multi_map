#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "multi_map_nav/NavigateToMapAction.h"

typedef actionlib::SimpleActionClient<multi_map_nav::NavigateToMapAction> Client;

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_client");
    
    if (argc != 6) {
        ROS_INFO("Usage: test_client <target_map> <x> <y> <qz> <qw>");
        return 1;
    }
    
    Client client("navigate_to_map", true);
    client.waitForServer();
    
    multi_map_nav::NavigateToMapGoal goal;
    goal.target_map = argv[1];
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = atof(argv[2]);
    goal.target_pose.pose.position.y = atof(argv[3]);
    goal.target_pose.pose.orientation.z = atof(argv[4]);
    goal.target_pose.pose.orientation.w = atof(argv[5]);
    
    client.sendGoal(goal);
    client.waitForResult();
    
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Navigation succeeded!");
    } else {
        ROS_INFO("Navigation failed: %s", client.getState().toString().c_str());
    }
    
    return 0;
}