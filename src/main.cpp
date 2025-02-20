#include <rclcpp/rclcpp.hpp>

#include "node/node.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions();
    auto main_node = std::make_shared<CyphalROS2::BridgeNode>(options);
    rclcpp::spin(main_node);

    rclcpp::shutdown();
    return 0;
}
