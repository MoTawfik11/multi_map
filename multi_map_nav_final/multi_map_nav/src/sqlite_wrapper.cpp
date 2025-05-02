#include "multi_map_nav/sqlite_wrapper.h"
#include <ros/package.h>
#include <stdexcept>

SQLiteWrapper::SQLiteWrapper(const std::string& db_path) : db_(nullptr), db_path_(db_path) {
    if (!openDB()) {
        throw std::runtime_error("Failed to open database");
    }
}

SQLiteWrapper::~SQLiteWrapper() {
    closeDB();
}

bool SQLiteWrapper::openDB() {
    return sqlite3_open(db_path_.c_str(), &db_) == SQLITE_OK;
}

void SQLiteWrapper::closeDB() {
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool SQLiteWrapper::addWormhole(const Wormhole& wormhole) {
    std::string sql = "INSERT INTO wormholes (map1, map2, x1, y1, qz1, qw1, x2, y2, qz2, qw2, radius) "
                      "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        return false;
    }
    
    sqlite3_bind_text(stmt, 1, wormhole.map1.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, wormhole.map2.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_double(stmt, 3, wormhole.pose1.position.x);
    sqlite3_bind_double(stmt, 4, wormhole.pose1.position.y);
    sqlite3_bind_double(stmt, 5, wormhole.pose1.orientation.z);
    sqlite3_bind_double(stmt, 6, wormhole.pose1.orientation.w);
    sqlite3_bind_double(stmt, 7, wormhole.pose2.position.x);
    sqlite3_bind_double(stmt, 8, wormhole.pose2.position.y);
    sqlite3_bind_double(stmt, 9, wormhole.pose2.orientation.z);
    sqlite3_bind_double(stmt, 10, wormhole.pose2.orientation.w);
    sqlite3_bind_double(stmt, 11, wormhole.radius);
    
    bool result = sqlite3_step(stmt) == SQLITE_DONE;
    sqlite3_finalize(stmt);
    return result;
}

std::vector<Wormhole> SQLiteWrapper::getWormholesForMap(const std::string& map_name) {
    std::vector<Wormhole> wormholes;
    std::string sql = "SELECT * FROM wormholes WHERE map1 = ? OR map2 = ?;";
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        return wormholes;
    }
    
    sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, map_name.c_str(), -1, SQLITE_STATIC);
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        Wormhole wh;
        wh.id = sqlite3_column_int(stmt, 0);
        wh.map1 = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        wh.map2 = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        
        wh.pose1.position.x = sqlite3_column_double(stmt, 3);
        wh.pose1.position.y = sqlite3_column_double(stmt, 4);
        wh.pose1.orientation.z = sqlite3_column_double(stmt, 5);
        wh.pose1.orientation.w = sqlite3_column_double(stmt, 6);
        
        wh.pose2.position.x = sqlite3_column_double(stmt, 7);
        wh.pose2.position.y = sqlite3_column_double(stmt, 8);
        wh.pose2.orientation.z = sqlite3_column_double(stmt, 9);
        wh.pose2.orientation.w = sqlite3_column_double(stmt, 10);
        
        wh.radius = sqlite3_column_double(stmt, 11);
        wormholes.push_back(wh);
    }
    
    sqlite3_finalize(stmt);
    return wormholes;
}

Wormhole SQLiteWrapper::findWormhole(const std::string& current_map, const geometry_msgs::Pose& pose, double radius) {
    std::string sql = "SELECT * FROM wormholes WHERE "
                      "(map1 = ? AND SQRT(POW(? - x1, 2) + POW(? - y1, 2)) <= ?) OR "
                      "(map2 = ? AND SQRT(POW(? - x2, 2) + POW(? - y2, 2)) <= ?);";
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        return Wormhole();
    }
    
    sqlite3_bind_text(stmt, 1, current_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_double(stmt, 2, pose.position.x);
    sqlite3_bind_double(stmt, 3, pose.position.y);
    sqlite3_bind_double(stmt, 4, radius);
    sqlite3_bind_text(stmt, 5, current_map.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_double(stmt, 6, pose.position.x);
    sqlite3_bind_double(stmt, 7, pose.position.y);
    sqlite3_bind_double(stmt, 8, radius);
    
    Wormhole result;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        result.id = sqlite3_column_int(stmt, 0);
        result.map1 = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        result.map2 = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        
        result.pose1.position.x = sqlite3_column_double(stmt, 3);
        result.pose1.position.y = sqlite3_column_double(stmt, 4);
        result.pose1.orientation.z = sqlite3_column_double(stmt, 5);
        result.pose1.orientation.w = sqlite3_column_double(stmt, 6);
        
        result.pose2.position.x = sqlite3_column_double(stmt, 7);
        result.pose2.position.y = sqlite3_column_double(stmt, 8);
        result.pose2.orientation.z = sqlite3_column_double(stmt, 9);
        result.pose2.orientation.w = sqlite3_column_double(stmt, 10);
        
        result.radius = sqlite3_column_double(stmt, 11);
    }
    
    sqlite3_finalize(stmt);
    return result;
}