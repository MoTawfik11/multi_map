#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "multi_map_nav/NavigateToMapAction.h"
#include "multi_map_nav/navigator.h"

class NavigationServer {
public:
    NavigationServer(const std::string& name) : 
        as_(nh_, name, boost::bind(&NavigationServer::executeCB, this, _1), false),
        action_name_(name) {
        
        navigator_ = std::make_unique<Navigator>();
        as_.start();
        ROS_INFO("NavigationServer started");
    }

    void executeCB(const multi_map_nav::NavigateToMapGoalConstPtr &goal) {
        ros::Rate r(10);
        bool success = true;
        
        feedback_.current_map = navigator_->getCurrentMap();
        feedback_.status = "Processing goal";
        as_.publishFeedback(feedback_);
        
        // Check if we're already in the target map
        if (goal->target_map == feedback_.current_map) {
            // Directly navigate to goal
            feedback_.status = "Navigating within current map";
            as_.publishFeedback(feedback_);
            
            success = navigator_->navigateTo(goal->target_pose);
            
            if (success) {
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
        if (!navigator_->getRobotPose(robot_pose)) {
            result_.success = false;
            result_.message = "Failed to get robot pose";
            as_.setAborted(result_);
            return;
        }
        
        Wormhole nearest_wh = navigator_->findNearestWormhole(feedback_.current_map, robot_pose);
        
        if (nearest_wh.id == 0) {
            result_.success = false;
            result_.message = "No wormhole found near current position";
            as_.setAborted(result_);
            return;
        }
        
        // Check if this wormhole connects to target map
        std::string connected_map = (nearest_wh.map1 == feedback_.current_map) ? nearest_wh.map2 : nearest_wh.map1;
        if (connected_map != goal->target_map) {
            result_.success = false;
            result_.message = "Wormhole doesn't connect to target map";
            as_.setAborted(result_);
            return;
        }
        
        // Navigate to wormhole in current map
        feedback_.status = "Navigating to wormhole";
        as_.publishFeedback(feedback_);
        
        geometry_msgs::PoseStamped wh_pose;
        wh_pose.header.frame_id = "map";
        wh_pose.header.stamp = ros::Time::now();
        
        if (nearest_wh.map1 == feedback_.current_map) {
            wh_pose.pose = nearest_wh.pose1;
        } else {
            wh_pose.pose = nearest_wh.pose2;
        }
        
        success = navigator_->navigateTo(wh_pose);
        
        if (!success) {
            result_.success = false;
            result_.message = "Failed to reach wormhole";
            as_.setAborted(result_);
            return;
        }
        
        // Switch maps
        feedback_.status = "Switching maps";
        as_.publishFeedback(feedback_);
        
        success = navigator_->switchMap(goal->target_map);
        
        if (!success) {
            result_.success = false;
            result_.message = "Failed to switch maps";
            as_.setAborted(result_);
            return;
        }
        
        // Navigate to final goal in new map
        feedback_.status = "Navigating to final goal in new map";
        as_.publishFeedback(feedback_);
        
        success = navigator_->navigateTo(goal->target_pose);
        
        if (success) {
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
    actionlib::SimpleActionServer<multi_map_nav::NavigateToMapAction> as_;
    std::string action_name_;
    multi_map_nav::NavigateToMapFeedback feedback_;
    multi_map_nav::NavigateToMapResult result_;
    std::unique_ptr<Navigator> navigator_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_server");
    NavigationServer server("navigate_to_map");
    ros::spin();
    return 0;
}