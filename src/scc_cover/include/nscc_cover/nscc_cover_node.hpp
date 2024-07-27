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

class NSCCCoverNode : public rclcpp::Node
{
public:
    NSCCCoverNode();
    ~NSCCCoverNode() = default;
};

#endif // NSCC_COVER_NODE_HPP