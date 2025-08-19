#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>

#include <cyphal/cyphal.h>

#include "_translate_msg.hpp"
#include "common.hpp"

namespace CyphalROS2 {

template <class FromROSType, class ToCyphalType>
void ros_callback_to_cyphal(
    InterfacePtr& interface,
    CanardPortID port_id,
    CanardTransferID* transfer_id_ptr,
    const typename FromROSType::SharedPtr& ros_msg_ptr,
    const rclcpp::MessageInfo& msg_info
) {
    /*if (msg_info.get_rmw_message_info().publisher_gid == 0) {
        return;
    }*/
    auto cyphal_msg = translate_ros_msg<
        const typename FromROSType::SharedPtr&,
        typename ToCyphalType::Type
    >(ros_msg_ptr);
    interface->send_msg<ToCyphalType>(&cyphal_msg, port_id, transfer_id_ptr);
}

#define MATCH_TYPE_RTC(type_name, ros_type, cyphal_type)                                                        \
    if (type_id == type_name) {                                                                                 \
        auto cb = [interface, port_id, topic_name](                                                             \
            const typename ros_type::SharedPtr ros_msg,                                                         \
            const rclcpp::MessageInfo& msg_info                                                                 \
        ) {                                                                                                     \
            static CanardTransferID transfer_id = 0;                                                            \
            ros_callback_to_cyphal<ros_type, cyphal_type>(interface, port_id, &transfer_id, ros_msg, msg_info); \
        };                                                                                                      \
        auto sub_options_base = rclcpp::SubscriptionOptionsBase();                                              \
        sub_options_base.ignore_local_publications = true;                                                      \
        auto sub_options = rclcpp::SubscriptionOptions(sub_options_base);                                       \
        return node->create_subscription<ros_type>(topic_name, rclcpp::QoS(10), cb, sub_options);               \
    }

inline std::optional<rclcpp::SubscriptionBase::SharedPtr> create_ros_to_cyphal_connector(
    const std::string& type_id,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& topic_name,
    InterfacePtr& interface,
    const CanardPortID port_id
) {

    MATCH_TYPE_RTC("Float32",         std_msgs::msg::Float32, Real32)
    MATCH_TYPE_RTC("Angle",           std_msgs::msg::Float32, Angle)
    MATCH_TYPE_RTC("AngularVelocity", std_msgs::msg::Float32, AngularVelocity)
    MATCH_TYPE_RTC("Velocity",        std_msgs::msg::Float32, Velocity)
    MATCH_TYPE_RTC("Integer32",       std_msgs::msg::Int32, Integer32)

    return std::nullopt;
}

}
