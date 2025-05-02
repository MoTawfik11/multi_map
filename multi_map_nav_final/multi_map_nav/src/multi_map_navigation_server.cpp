#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sqlite3.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

class MultiMapNavigationServer
{
private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::ServiceServer switch_map_service_;
    sqlite3* db_;
    std::string current_map_;
    std::map<std::string, std::string> map_paths_; // {map_name: full_path}

public:
    MultiMapNavigationServer()
    {
        initDatabase();
        loadMapPaths();
        switch_map_service_ = nh_.advertiseService("switch_map", &MultiMapNavigationServer::switchMapCallback, this);
        ROS_INFO("Multi-Map Navigation Server Ready.");
    }

    ~MultiMapNavigationServer()
    {
        sqlite3_close(db_);
    }

    void initDatabase()
    {
        std::string db_path = ros::package::getPath("multi_map_nav") + "/config/wormholes.db";
        if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK)
        {
            ROS_ERROR("Failed to open wormhole database: %s", sqlite3_errmsg(db_));
            exit(1);
        }
    }

    void loadMapPaths()
    {
        std::string base_path = ros::package::getPath("multi_map_nav") + "/maps/";
        for (int i = 1; i <= 6; ++i)
        {
            std::string name = "room" + std::to_string(i);
            std::string path = base_path + name + ".yaml";
            map_paths_[name] = path;
        }
    }

    bool switchMapCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        std::string target_map = getNextMap(current_map_);
        if (target_map.empty())
        {
            ROS_WARN("No next map found.");
            return false;
        }

        std::string cmd = "rosrun map_server map_server " + map_paths_[target_map];
        system(("rosnode kill /map_server && " + cmd + " &").c_str());
        current_map_ = target_map;
        ROS_INFO("Switched to map: %s", current_map_.c_str());
        return true;
    }

    std::string getNextMap(const std::string& current)
    {
        if (current.empty()) return "room1";
        std::string base = "room";
        int next = std::stoi(current.substr(4)) % 6 + 1;
        return base + std::to_string(next);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_map_navigation_server");
    MultiMapNavigationServer server;
    ros::spin();
    return 0;
}
