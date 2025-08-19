#pragma once

#include <chrono>
#include <thread>
#include <optional>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <voltbro/ros/ros2.hpp>
#include <cyphal/cyphal.h>

#include "_translate_msg.hpp"

namespace CyphalROS2 {

using namespace std::chrono_literals;

template <class CyphalRequestType, class CyphalResponseType, class ROSType>
class CyphalServiceBackend
    : public ROS2ServiceProvider<ROSType>
    , public AbstractSubscription<CyphalResponseType>
{
protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::map<
        CanardTransferID,
        std::optional<std::pair<
            std::shared_ptr<typename CyphalResponseType::Type>,
            CanardRxTransfer  // FIXME: avoid copying
        >>
    > requests_info;
    std::mutex requests_info_lock;
    InterfacePtr interface; // FIXME: should use interface from AbstractSubscription

    CanardNodeID target_node_id;
    CanardTransferID transfer_id = 0;

public:
    CyphalServiceBackend(
        std::shared_ptr<rclcpp::Node> node,
        const std::string& service_name,
        InterfacePtr& interface,
        CanardPortID port_id,
        CanardNodeID target_node_id
    )
        : interface(interface)
        , node_(node)
        , ROS2ServiceProvider<ROSType>(node, service_name)
        , AbstractSubscription<CyphalResponseType>(interface, port_id, CanardTransferKindResponse)
        , target_node_id(target_node_id)
    {}

    void callback(
        const std::shared_ptr<typename ROSType::Request> ros_request,
        std::shared_ptr<typename ROSType::Response> ros_response
    ) override {
        RCLCPP_DEBUG(node_->get_logger(), "Got request, sending to Cyphal");

        typename CyphalRequestType::Type cyphal_request = translate_ros_msg<
            const std::shared_ptr<typename ROSType::Request>&,
            typename CyphalRequestType::Type
        >(ros_request);

        CanardTransferID current_transfer_id = transfer_id;
        RCLCPP_DEBUG(node_->get_logger(), "Saving transfer_id <%u>", current_transfer_id);

        {
            std::lock_guard<std::mutex> lock(requests_info_lock);
            requests_info[current_transfer_id] = std::nullopt;
        }
        interface->send_request<CyphalRequestType>(
            &cyphal_request,
            AbstractSubscription<CyphalResponseType>::port_id,
            &transfer_id,
            target_node_id
        );

        bool is_ok = false;
        size_t wait_counter = 0;
        while (wait_counter < 11) {
            {
                std::lock_guard<std::mutex> lock(requests_info_lock);
                if (!requests_info.count(current_transfer_id)) {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Response slot for <%u> deleted before handling",
                                 current_transfer_id);
                    return;
                }
                if (!requests_info[current_transfer_id]) {
                    // Not ready yet
                } else {
                    RCLCPP_DEBUG(node_->get_logger(), "Translating response for <%u>", current_transfer_id);
                    auto [cyphal_response, transfer_obj] =
                        requests_info[current_transfer_id].value();
                    *ros_response = translate_cyphal_msg<
                        const std::shared_ptr<typename CyphalResponseType::Type>&,
                        typename ROSType::Response
                    >(cyphal_response, &transfer_obj);
                    is_ok = true;
                    break;
                }
            }
            std::this_thread::sleep_for(10ms);
            wait_counter++;
        }

        {
            std::lock_guard<std::mutex> lock(requests_info_lock);
            requests_info.erase(current_transfer_id);
        }

        if (!is_ok) {
            RCLCPP_WARN(node_->get_logger(), "No response for transfer_id <%u>", current_transfer_id);
        }
    }

    // Cyphal callback
    void handler(
        const std::shared_ptr<typename CyphalResponseType::Type>& msg,
        CanardRxTransfer* transfer
    ) override {
        CanardTransferID tid = transfer->metadata.transfer_id;

        RCLCPP_DEBUG(node_->get_logger(), "Got transfer_id <%u>", tid);
        std::lock_guard<std::mutex> lock(requests_info_lock);
        if (!requests_info.count(tid)) {
            RCLCPP_ERROR(node_->get_logger(), "Response refers to non-existent or timed out transfer_id <%u>", tid);
            return;
        }
        if (requests_info[tid]) {
            RCLCPP_ERROR(node_->get_logger(), "Received repeated response for transfer_id <%u>", tid);
            return;
        }
        RCLCPP_DEBUG(node_->get_logger(), "Saving response from Cyphal for <%u>", tid);
        requests_info[tid] = {msg, *transfer};
    }
};

#define MATCH_TYPE_SRV(type_name, cyphal_request_type, cyphal_response_type, ros_type)             \
    if (type_id == type_name) {                                                                    \
        return std::make_unique<                                                                   \
            CyphalServiceBackend<cyphal_request_type, cyphal_response_type, ros_type>              \
        >(node, service_name, interface, port_id, target_id);                                      \
    }

inline std::unique_ptr<TransferListener> create_ros_service(
    const std::string& type_id,
    std::shared_ptr<rclcpp::Node> node,
    const std::string& service_name,
    InterfacePtr& interface,
    CanardPortID port_id,
    CanardNodeID target_id
) {
    MATCH_TYPE_SRV("HMI.Led", LEDServiceRequest, LEDServiceResponse, cyphal_ros2_bridge::srv::CallHMILed)
    MATCH_TYPE_SRV("HMI.Beeper", BeeperServiceRequest, BeeperServiceResponse, cyphal_ros2_bridge::srv::CallHMIBeeper)
    MATCH_TYPE_SRV("Echo", EchoRequest, EchoResponse, cyphal_ros2_bridge::srv::Echo)

    return nullptr;
}

}
