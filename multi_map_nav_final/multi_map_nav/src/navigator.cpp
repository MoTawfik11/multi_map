#include "multi_map_nav/navigator.h"
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

Navigator::Navigator() : 
    move_base_client_(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true)),
    tf_listener_(tf_buffer_) {
    
    // Initialize database
    std::string db_path = ros::package::getPath("multi_map_nav") + "/wormholes.db";
    sqlite_wrapper_ = std::make_unique<SQLiteWrapper>(db_path);
    
    // Wait for move_base server
    ROS_INFO("Waiting for move_base action server...");
    move_base_client_->waitForServer();
    ROS_INFO("move_base action server connected!");
    
    // Get current map
    if (!nh_.getParam("/current_map", current_map_)) {
        current_map_ = "room1";
        nh_.setParam("/current_map", current_map_);
    }
}

bool Navigator::getRobotPose(geometry_msgs::PoseStamped& pose) {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            "map", "base_link", ros::Time(0), ros::Duration(1.0));
        
        pose.header.stamp = transform.header.stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to get robot pose: %s", ex.what());
        return false;
    }
}

bool Navigator::navigateTo(const geometry_msgs::PoseStamped& goal) {
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = goal;
    mb_goal.target_pose.header.stamp = ros::Time::now();
    
    move_base_client_->sendGoal(mb_goal);
    move_base_client_->waitForResult();
    
    return move_base_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool Navigator::switchMap(const std::string& new_map) {
    ros::ServiceClient clear_costmaps = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    ros::ServiceClient switch_map = nh_.serviceClient<std_srvs::Empty>("/change_map");
    
    std_srvs::Empty srv;
    if (!clear_costmaps.call(srv)) {
        ROS_WARN("Failed to clear costmaps");
        return false;
    }
    
    current_map_ = new_map;
    nh_.setParam("/current_map", current_map_);
    
    if (!switch_map.call(srv)) {
        ROS_WARN("Failed to switch maps");
        return false;
    }
    
    return true;
}

Wormhole Navigator::findNearestWormhole(const std::string& current_map, const geometry_msgs::PoseStamped& pose) {
    geometry_msgs::Pose robot_pose = pose.pose;
    return sqlite_wrapper_->findWormhole(current_map, robot_pose, 1.0);
}

std::string Navigator::getCurrentMap() const {
    return current_map_;
}