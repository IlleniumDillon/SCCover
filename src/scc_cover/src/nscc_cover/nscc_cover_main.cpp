#include <rclcpp/rclcpp.hpp>

#include "nscc_cover_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NSCCCoverNode>());
    rclcpp::shutdown();
    return 0;
}