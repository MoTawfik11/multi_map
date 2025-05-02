#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <multi_map_nav/MultiMapNavigationAction.h>
#include "multi_map_nav/sqlite_wrapper.h"

class MultiMapNavigation {
public:
    MultiMapNavigation(const std::string& name) : 
        as_(nh_, name, boost::bind(&MultiMapNavigation::executeCB, this, _1), false),
        action_name_(name) {
        
        // Initialize database
        std::string db_path = ros::package::getPath("multi_map_nav") + "/wormholes.db";
        sqlite_wrapper_ = std::make_unique<SQLiteWrapper>(db_path);
        
        // Initialize move_base client
        mb_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        
        // Wait for move_base server
        ROS_INFO("Waiting for move_base action server...");
        mb_client_->waitForServer();
        ROS_INFO("move_base action server connected!");
        
        // Start action server
        as_.start();
        ROS_INFO("MultiMapNavigation action server started");
        
        // Get current map name from parameter server
        if (!nh_.getParam("/current_map", current_map_)) {
            current_map_ = "room1";
            nh_.setParam("/current_map", current_map_);
        }
        
        // Setup TF listener
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    }

    void executeCB(const wormhole_navigation::MultiMapNavigationGoalConstPtr &goal) {
        ros::Rate r(10);
        bool success = true;
        
        feedback_.current_map = current_map_;
        feedback_.status = "Processing goal";
        as_.publishFeedback(feedback_);
        
        // Check if we're already in the target map
        if (goal->target_map == current_map_) {
            // Directly send goal to move_base
            move_base_msgs::MoveBaseGoal mb_goal;
            mb_goal.target_pose = goal->target_pose;
            mb_goal.target_pose.header.stamp = ros::Time::now();
            mb_goal.target_pose.header.frame_id = "map";
            
            mb_client_->sendGoal(mb_goal);
            
            feedback_.status = "Navigating within current map";
            as_.publishFeedback(feedback_);
            
            mb_client_->waitForResult();
            
            if (mb_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                result_.success = true;
                result_.message = "Goal reached within current map";
                as_.setSucceeded(result_);
            } else {
                result_.success = false;
                result_.message = "Failed to reach goal within current map";
                as_.setAborted(result_);
            }
            return;
        }
        
        // Need to switch maps - find nearest wormhole
        geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            result_.success = false;
            result_.message = "Failed to get robot pose";
            as_.setAborted(result_);
            return;
        }
        
        Wormhole nearest_wh = sqlite_wrapper_->find_wormhole(
            current_map_, 
            robot_pose.pose.position.x, 
            robot_pose.pose.position.y, 
            1.0  // search radius
        );
        
        if (nearest_wh.id == 0) {
            result_.success = false;
            result_.message = "No wormhole found near current position";
            as_.setAborted(result_);
            return;
        }
        
        // Check if this wormhole connects to target map
        std::string connected_map = (nearest_wh.map1 == current_map_) ? nearest_wh.map2 : nearest_wh.map1;
        if (connected_map != goal->target_map) {
            result_.success = false;
            result_.message = "Wormhole doesn't connect to target map";
            as_.setAborted(result_);
            return;
        }
        
        // Navigate to wormhole in current map
        feedback_.status = "Navigating to wormhole";
        as_.publishFeedback(feedback_);
        
        move_base_msgs::MoveBaseGoal mb_goal;
        mb_goal.target_pose.header.frame_id = "map";
        mb_goal.target_pose.header.stamp = ros::Time::now();
        
        if (nearest_wh.map1 == current_map_) {
            mb_goal.target_pose.pose.position.x = nearest_wh.x1;
            mb_goal.target_pose.pose.position.y = nearest_wh.y1;
        } else {
            mb_goal.target_pose.pose.position.x = nearest_wh.x2;
            mb_goal.target_pose.pose.position.y = nearest_wh.y2;
        }
        mb_goal.target_pose.pose.orientation.w = 1.0;  // Default orientation
        
        mb_client_->sendGoal(mb_goal);
        mb_client_->waitForResult();
        
        if (mb_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            result_.success = false;
            result_.message = "Failed to reach wormhole";
            as_.setAborted(result_);
            return;
        }
        
        // Switch maps
        feedback_.status = "Switching maps";
        as_.publishFeedback(feedback_);
        
        if (!switchMaps(goal->target_map)) {
            result_.success = false;
            result_.message = "Failed to switch maps";
            as_.setAborted(result_);
            return;
        }
        
        // Navigate to final goal in new map
        feedback_.status = "Navigating to final goal in new map";
        as_.publishFeedback(feedback_);
        
        mb_goal.target_pose = goal->target_pose;
        mb_goal.target_pose.header.stamp = ros::Time::now();
        mb_goal.target_pose.header.frame_id = "map";
        
        mb_client_->sendGoal(mb_goal);
        mb_client_->waitForResult();
        
        if (mb_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            result_.success = true;
            result_.message = "Goal reached successfully";
            as_.setSucceeded(result_);
        } else {
            result_.success = false;
            result_.message = "Failed to reach goal after map switch";
            as_.setAborted(result_);
        }
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<wormhole_navigation::MultiMapNavigationAction> as_;
    std::string action_name_;
    wormhole_navigation::MultiMapNavigationFeedback feedback_;
    wormhole_navigation::MultiMapNavigationResult result_;
    
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> mb_client_;
    std::unique_ptr<SQLiteWrapper> sqlite_wrapper_;
    
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::string current_map_;
    
    bool getRobotPose(geometry_msgs::PoseStamped& pose) {
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
    
    bool switchMaps(const std::string& new_map) {
        // Call map switching service
        ros::ServiceClient clear_costmaps = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        ros::ServiceClient switch_map = nh_.serviceClient<std_srvs::Empty>("/change_map");
        
        std_srvs::Empty srv;
        if (!clear_costmaps.call(srv)) {
            ROS_WARN("Failed to clear costmaps");
            return false;
        }
        
        // Update current map parameter
        current_map_ = new_map;
        nh_.setParam("/current_map", current_map_);
        
        // In a real implementation, you would also change the map being used by the navigation stack
        // This typically involves calling a service to change the map file
        if (!switch_map.call(srv)) {
            ROS_WARN("Failed to switch maps");
            return false;
        }
        
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_map_navigation");
    MultiMapNavigation navigation("multi_map_navigation");
    ros::spin();
    return 0;
}