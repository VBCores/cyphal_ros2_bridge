#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include "node/node.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions();

    std::string config_file_name;
    //nh->getParam("config_file", config_file_name);
    //RCLCPP_INFO_STERAM("Using <" << config_file_name << ">");
    std::ifstream config_file_stream(config_file_name);
    json config_json = json::parse(config_file_stream);

    auto main_node = std::make_shared<CyphalROS2::BridgeNode>(options, config_json);
    rclcpp::spin(main_node);

    rclcpp::shutdown();
    return 0;
}
