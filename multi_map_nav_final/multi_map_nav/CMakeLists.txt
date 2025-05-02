cmake_minimum_required(VERSION 3.0.2)
project(multi_map_nav)

# Enable C++17 support
set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
  roslib
)

# Find the SQLite3 package
find_package(SQLite3 REQUIRED)

# Declare custom action files
add_action_files(
  DIRECTORY action
  FILES NavigateToMap.action
)

# Generate added messages and actions
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  nav_msgs
)

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    actionlib 
    actionlib_msgs 
    message_runtime
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
)

# Build SQLite wrapper library
add_library(sqlite_wrapper
  src/sqlite_wrapper.cpp
)
target_link_libraries(sqlite_wrapper
  ${catkin_LIBRARIES}
  ${SQLite3_LIBRARIES}
)

# Build navigator library
add_library(navigator
  src/navigator.cpp
)
target_link_libraries(navigator
  sqlite_wrapper
  ${catkin_LIBRARIES}
  ${SQLite3_LIBRARIES}
)

# Build the navigation server
add_executable(navigation_server
  src/navigation_server.cpp
)
add_dependencies(navigation_server 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(navigation_server
  navigator
  sqlite_wrapper
  ${catkin_LIBRARIES}
  ${SQLite3_LIBRARIES}
)

# Build the test client
add_executable(test_client 
  src/test_client.cpp
)
add_dependencies(test_client 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_client
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS 
  navigation_server 
  test_client 
  navigator 
  sqlite_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install include files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)