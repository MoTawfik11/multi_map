#ifndef MULTI_MAP_NAV_SQLITE_WRAPPER_H
#define MULTI_MAP_NAV_SQLITE_WRAPPER_H

#include <string>
#include <vector>
#include <sqlite3.h>
#include <geometry_msgs/Pose.h>

struct Wormhole {
    int id;
    std::string map1;
    std::string map2;
    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;
    double radius;
};

class SQLiteWrapper {
public:
    SQLiteWrapper(const std::string& db_path);
    ~SQLiteWrapper();
    
    bool addWormhole(const Wormhole& wormhole);
    bool removeWormhole(int id);
    std::vector<Wormhole> getWormholesForMap(const std::string& map_name);
    Wormhole findWormhole(const std::string& current_map, const geometry_msgs::Pose& pose, double radius);
    
private:
    sqlite3* db_;
    std::string db_path_;
    
    bool openDB();
    void closeDB();
};

#endif // MULTI_MAP_NAV_SQLITE_WRAPPER_H