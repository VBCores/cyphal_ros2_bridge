#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <json.hpp>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>

#include "common.hpp"

using json = nlohmann::json;

namespace CyphalROS2 {

enum class ROSType {
    TOPIC,
    SERVICE
};

enum class ROSDirection {
    READ,
    WRITE,
    BI
};

class BridgeNode : public rclcpp::Node {
private:
    json config_json;
    std::shared_ptr<CyphalInterface> interface;

    rclcpp::TimerBase::SharedPtr hbeat_timer;

    std::map<CanardPortID, std::unique_ptr<IMultipleListener>> cyphal_subscriptions;
    std::vector<std::unique_ptr<TransferListener>> cyphal_listeners;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> ros_subscriptions;

    void add_connection(const json& connection);
    void hbeat_cb();
    void parsing_error(const std::string& error);

    BridgeNode(const rclcpp::NodeOptions& options);

public:
    std::shared_ptr<BridgeNode> get_shared() {
        return std::static_pointer_cast<BridgeNode>(shared_from_this());
    }
    static std::shared_ptr<BridgeNode> create_bridge(const rclcpp::NodeOptions& options);
    ~BridgeNode();
};

}
