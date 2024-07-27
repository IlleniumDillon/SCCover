#ifndef NSCC_COVER_NODE_HPP
#define NSCC_COVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/utils.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#include "scc_message/msg/polygon_map.hpp"

#include "opencv2/opencv.hpp"

class NSCCCoverNode : public rclcpp::Node
{
public:
    NSCCCoverNode();
    ~NSCCCoverNode() = default;
public:
    // config
    double scene_width_;
    double scene_height_;
    double position_resolution_;
    // map update
    scc_message::msg::PolygonMap pmap;
    nav_msgs::msg::OccupancyGrid gmap;

    rclcpp::Subscription<scc_message::msg::PolygonMap>::SharedPtr sub_polygon_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_map_;
    // plan

public:
    // config
    void loadConfig();
    // map update
    void polygonMapCallback(const scc_message::msg::PolygonMap::SharedPtr msg);
};

#endif // NSCC_COVER_NODE_HPP