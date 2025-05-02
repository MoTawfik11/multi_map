#ifndef MULTI_MAP_NAV_NAVIGATOR_H
#define MULTI_MAP_NAV_NAVIGATOR_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "multi_map_nav/sqlite_wrapper.h"

class Navigator {
public:
    Navigator();
    
    bool getRobotPose(geometry_msgs::PoseStamped& pose);
    bool navigateTo(const geometry_msgs::PoseStamped& goal);
    bool switchMap(const std::string& new_map);
    Wormhole findNearestWormhole(const std::string& current_map, const geometry_msgs::PoseStamped& pose);
    std::string getCurrentMap() const;
    
private:
    ros::NodeHandle nh_;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;
    std::unique_ptr<SQLiteWrapper> sqlite_wrapper_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string current_map_;
};

#endif // MULTI_MAP_NAV_NAVIGATOR_H