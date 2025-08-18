#pragma once

#include <memory>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <cyphal/cyphal.h>
#include <cyphal/subscriptions/subscription.h>

#include "_translate_msg.hpp"
#include "common.hpp"

namespace CyphalROS2 {

template <class FromCyphalType, class ToROSType>
class CyphalSubscriptionToROS
    : public AbstractSubscription<FromCyphalType>
    , public IMultipleListener
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::map<CanardNodeID, typename rclcpp::Publisher<ToROSType>::SharedPtr> publishers_;

public:
    CyphalSubscriptionToROS(
        const std::shared_ptr<rclcpp::Node>& node,
        const std::string& topic_name,
        InterfacePtr& interface,
        CanardPortID port_id,
        CanardNodeID source_node_id
    )
        : AbstractSubscription<FromCyphalType>(interface, port_id, CanardTransferKindMessage)
        , node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "Publishing topic <%s>", topic_name.c_str());
        publishers_[source_node_id] =
            node_->create_publisher<ToROSType>(topic_name, rclcpp::QoS(rclcpp::KeepLast(5)));
    }

    void add_listener(const std::string& topic, CanardNodeID source_node_id) override {
        RCLCPP_INFO(node_->get_logger(), "Publishing topic <%s>", topic.c_str());
        publishers_[source_node_id] =
            node_->create_publisher<ToROSType>(topic, rclcpp::QoS(rclcpp::KeepLast(5)));
    }

    void handler(const std::shared_ptr<typename FromCyphalType::Type>& cyphal_msg_ptr,
                 CanardRxTransfer* transfer) override
    {
        const auto current_id = transfer->metadata.remote_node_id;

        const bool has_specific = publishers_.count(current_id) != 0;
        const bool has_generic  = publishers_.count(0) != 0;
        if (!has_specific && !has_generic) {
            return;
        }

        auto ros_msg = translate_cyphal_msg<
            const std::shared_ptr<typename FromCyphalType::Type>&,
            ToROSType
        >(cyphal_msg_ptr, transfer);

        if (has_specific) {
            publishers_[current_id]->publish(ros_msg);
        } else {
            publishers_[0]->publish(ros_msg);
        }
    }
};

#define MATCH_TYPE_CTR(type_name, cyphal_type, ros_type)                          \
    if (type_id == type_name) {                                                   \
        return std::make_unique<                                                  \
            CyphalSubscriptionToROS<cyphal_type, ros_type>                        \
        >(node, topic_name, interface, port_id, source_node_id);                  \
    }

inline std::unique_ptr<IMultipleListener> create_cyphal_to_ros_connector(
    const std::string& type_id,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& topic_name,
    InterfacePtr& interface,
    CanardPortID port_id,
    CanardNodeID source_node_id
) {
    MATCH_TYPE_CTR("Battery",          VBBatteryState,   sensor_msgs::msg::BatteryState)
    MATCH_TYPE_CTR("Float32",          Real32,           std_msgs::msg::Float32)
    MATCH_TYPE_CTR("Diagnostic",       DiagnosticRecord, diagnostic_msgs::msg::DiagnosticStatus)
    MATCH_TYPE_CTR("Angle",            Angle,            std_msgs::msg::Float32)
    MATCH_TYPE_CTR("AngularVelocity",  AngularVelocity,  std_msgs::msg::Float32)
    MATCH_TYPE_CTR("Velocity",         Velocity,         std_msgs::msg::Float32)
    MATCH_TYPE_CTR("Integer32",        Integer32,        std_msgs::msg::Int32)

    return std::unique_ptr<IMultipleListener>(nullptr);
}

}
