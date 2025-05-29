# Multi-Map Navigation in ROS Noetic

This ROS Noetic package implements **multi-map navigation** using a wormhole mechanism to transition between separate maps. It is ideal for applications such as indoor mobile robots operating in large environments divided into multiple rooms or floors.

## Features

- 🗺️ Support for multiple maps
- 🕳️ Wormhole-based transition between maps
- 🧠 SQLite3 database for storing wormhole positions
- 🤖 ROS action server (`multi_map_navigation_server`) for managing navigation goals across maps
- ✅ Compatible with `move_base`, `amcl`, and standard ROS navigation stack

## Architecture

- Each room or environment is mapped separately using `gmapping` or `cartographer`.
- Wormholes are overlapping regions between maps that serve as transition points.
- The system queries a local SQLite3 database to determine when and where to switch maps.
- A C++ ROS node manages switching maps and relaying navigation goals accordingly.

## Dependencies

- ROS Noetic
- `sqlite3` (for database interaction)
- `tf2`, `move_base`, `amcl`, `map_server`, `nav_msgs`, `geometry_msgs`
- OpenCV (if using image-based map parsing)
- `actionlib` (for the navigation server)

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/MoTawfik11/multi_map.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
