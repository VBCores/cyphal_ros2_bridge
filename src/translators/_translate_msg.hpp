#pragma once

#include <string>
#include <boost/format.hpp>

#include <cyphal/cyphal.h>

#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <uavcan/si/unit/velocity/Scalar_1_0.h>
#include <std_msgs/msg/float32.hpp>

#include <uavcan/primitive/scalar/Integer32_1_0.h>
#include <std_msgs/msg/int32.hpp>

#include <voltbro/battery/state_1_0.h>
#include <sensor_msgs/msg/battery_state.hpp>

#include <voltbro/battery/buttons_1_0.h>
#include <cyphal_ros2_bridge/msg/power_buttons.hpp>

#include <voltbro/hmi/beeper_service_1_0.h>
#include <cyphal_ros2_bridge/srv/call_hmi_beeper.hpp>

#include <voltbro/hmi/led_service_1_0.h>
#include <cyphal_ros2_bridge/srv/call_hmi_led.hpp>

#include <uavcan/diagnostic/Record_1_1.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

TYPE_ALIAS(Integer32, uavcan_primitive_scalar_Integer32_1_0)
TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
TYPE_ALIAS(Velocity, uavcan_si_unit_velocity_Scalar_1_0)
TYPE_ALIAS(AngularVelocity, uavcan_si_unit_angular_velocity_Scalar_1_0)
TYPE_ALIAS(VBBatteryState, voltbro_battery_state_1_0)
TYPE_ALIAS(VBPowerButtons, voltbro_battery_buttons_1_0)
TYPE_ALIAS(DiagnosticRecord, uavcan_diagnostic_Record_1_1)
TYPE_ALIAS(Angle, uavcan_si_unit_angle_Scalar_1_0)
TYPE_ALIAS(LEDServiceRequest, voltbro_hmi_led_service_Request_1_0)
TYPE_ALIAS(LEDServiceResponse, voltbro_hmi_led_service_Response_1_0)
TYPE_ALIAS(BeeperServiceRequest, voltbro_hmi_beeper_service_Request_1_0)
TYPE_ALIAS(BeeperServiceResponse, voltbro_hmi_beeper_service_Response_1_0)

namespace CyphalROS2 {

template <class FromA, class ToB>
inline ToB translate_ros_msg(FromA);

template <class FromA, class ToB>
inline ToB translate_cyphal_msg(FromA, CanardRxTransfer*);

// ---------- HMI LED service ----------

template <>
inline LEDServiceRequest::Type translate_ros_msg(
    const cyphal_ros2_bridge::srv::CallHMILed::Request& ros_request
) {
    auto cyphal_request = LEDServiceRequest::Type();
    cyphal_request.r.value = ros_request.led.r;
    cyphal_request.g.value = ros_request.led.g;
    cyphal_request.b.value = ros_request.led.b;
    cyphal_request.interface.value = ros_request.led.interface;
    cyphal_request.duration.second = ros_request.led.duration;
    cyphal_request.frequency.hertz = ros_request.led.frequency;
    return cyphal_request;
}

template <>
inline cyphal_ros2_bridge::srv::CallHMILed::Response translate_cyphal_msg(
    const std::shared_ptr<LEDServiceResponse::Type>& cyphal_response, CanardRxTransfer* /*transfer*/
) {
    auto ros_response = cyphal_ros2_bridge::srv::CallHMILed::Response();
    ros_response.accepted = cyphal_response->accepted.value;
    return ros_response;
}

// ---------- HMI Beeper service ----------

template <>
inline BeeperServiceRequest::Type translate_ros_msg(
    const cyphal_ros2_bridge::srv::CallHMIBeeper::Request& ros_request
) {
    auto cyphal_request = BeeperServiceRequest::Type();
    cyphal_request.duration.second = ros_request.beeper.duration;
    cyphal_request.frequency.hertz = ros_request.beeper.frequency;
    return cyphal_request;
}

template <>
inline cyphal_ros2_bridge::srv::CallHMIBeeper::Response translate_cyphal_msg(
    const std::shared_ptr<BeeperServiceResponse::Type>& cyphal_response, CanardRxTransfer* /*transfer*/
) {
    auto ros_response = cyphal_ros2_bridge::srv::CallHMIBeeper::Response();
    ros_response.accepted = cyphal_response->accepted.value;
    return ros_response;
}

// ---------- Scalar Float32 <-> Real32 ----------

template <>
inline Real32::Type translate_ros_msg(const std::shared_ptr<std_msgs::msg::Float32>& ros_msg) {
    auto cyphal_msg = Real32::Type();
    cyphal_msg.value = ros_msg->data;
    return cyphal_msg;
}

template <>
inline std_msgs::msg::Float32 translate_cyphal_msg(
    const std::shared_ptr<Real32::Type>& cyphal_msg, CanardRxTransfer* /*transfer*/
) {
    auto ros_msg = std_msgs::msg::Float32();
    ros_msg.data = cyphal_msg->value;
    return ros_msg;
}

// ---------- Scalar Integer32 <-> Int32 ----------

template <>
inline Integer32::Type translate_ros_msg(const std::shared_ptr<std_msgs::msg::Int32>& ros_msg) {
    auto cyphal_msg = Integer32::Type();
    cyphal_msg.value = ros_msg->data;
    return cyphal_msg;
}

template <>
inline std_msgs::msg::Int32 translate_cyphal_msg(
    const std::shared_ptr<Integer32::Type>& cyphal_msg, CanardRxTransfer* /*transfer*/
) {
    auto ros_msg = std_msgs::msg::Int32();
    ros_msg.data = cyphal_msg->value;
    return ros_msg;
}

// ---------- Angle ----------

template <>
inline Angle::Type translate_ros_msg(const std::shared_ptr<std_msgs::msg::Float32>& ros_msg) {
    auto cyphal_msg = Angle::Type();
    cyphal_msg.radian = ros_msg->data;
    return cyphal_msg;
}

template <>
inline std_msgs::msg::Float32 translate_cyphal_msg(
    const std::shared_ptr<Angle::Type>& cyphal_msg, CanardRxTransfer* /*transfer*/
) {
    auto ros_msg = std_msgs::msg::Float32();
    ros_msg.data = cyphal_msg->radian;
    return ros_msg;
}

// ---------- Angular Velocity ----------

template <>
inline AngularVelocity::Type translate_ros_msg(const std::shared_ptr<std_msgs::msg::Float32>& ros_msg) {
    auto cyphal_msg = AngularVelocity::Type();
    cyphal_msg.radian_per_second = ros_msg->data;
    return cyphal_msg;
}

template <>
inline std_msgs::msg::Float32 translate_cyphal_msg(
    const std::shared_ptr<AngularVelocity::Type>& cyphal_msg, CanardRxTransfer* /*transfer*/
) {
    auto ros_msg = std_msgs::msg::Float32();
    ros_msg.data = cyphal_msg->radian_per_second;
    return ros_msg;
}

// ---------- Linear Velocity ----------

template <>
inline Velocity::Type translate_ros_msg(const std::shared_ptr<std_msgs::msg::Float32>& ros_msg) {
    auto cyphal_msg = Velocity::Type();
    cyphal_msg.meter_per_second = ros_msg->data;
    return cyphal_msg;
}

template <>
inline std_msgs::msg::Float32 translate_cyphal_msg(
    const std::shared_ptr<Velocity::Type>& cyphal_msg, CanardRxTransfer* /*transfer*/
) {
    auto ros_msg = std_msgs::msg::Float32();
    ros_msg.data = cyphal_msg->meter_per_second;
    return ros_msg;
}

// ---------- Battery State ----------

template <>
inline sensor_msgs::msg::BatteryState translate_cyphal_msg(
    const std::shared_ptr<VBBatteryState::Type>& bat_info,
    CanardRxTransfer* /*transfer*/
) {
    sensor_msgs::msg::BatteryState battery;

    battery.voltage = bat_info->voltage.volt;
    battery.current = bat_info->current.ampere;
    battery.charge = bat_info->charge.coulomb / 3.6f;
    battery.capacity = bat_info->capacity.coulomb / 3.6f;
    battery.percentage = (battery.capacity > 0.0f) ? (battery.charge / battery.capacity) : 0.0f;

    battery.design_capacity = bat_info->design_capacity.coulomb / 3.6f;

    battery.power_supply_status = bat_info->power_supply_status.value;
    battery.power_supply_health = bat_info->power_supply_health.value;
    battery.power_supply_technology = bat_info->power_supply_technology.value;
    battery.present = (bat_info->is_present.value == 1);

    battery.location = std::string(
        reinterpret_cast<const char*>(bat_info->location.value.elements),
        bat_info->location.value.count
    );
    battery.serial_number = std::string(
        reinterpret_cast<const char*>(bat_info->serial_number.value.elements),
        bat_info->serial_number.value.count
    );

    return battery;
}

// ---------- Power Buttons ----------

template <>
inline cyphal_ros2_bridge::msg::PowerButtons translate_cyphal_msg(
    const std::shared_ptr<VBPowerButtons::Type>& buttons_info,
    CanardRxTransfer* transfer
) {
    cyphal_ros2_bridge::msg::PowerButtons buttons;
    buttons.emergency = buttons_info->emergency_button.value;
    buttons.user = buttons_info->user_button.value;
    return buttons;
}

// ---------- Diagnostics ----------

template <>
inline diagnostic_msgs::msg::DiagnosticStatus translate_cyphal_msg(
    const std::shared_ptr<DiagnosticRecord::Type>& diagnostic,
    CanardRxTransfer* transfer
) {
    diagnostic_msgs::msg::DiagnosticStatus ros_diagnostic;

    ros_diagnostic.name = (boost::format("Node %1%") % transfer->metadata.remote_node_id).str();
    ros_diagnostic.hardware_id = (boost::format("%1%") % transfer->metadata.remote_node_id).str();
    ros_diagnostic.message = reinterpret_cast<const char*>(diagnostic->text.elements);

    uint8_t severity;
    switch (diagnostic->severity.value) {
        case uavcan_diagnostic_Severity_1_0_TRACE:
        case uavcan_diagnostic_Severity_1_0_DEBUG:
        case uavcan_diagnostic_Severity_1_0_INFO:
            severity = diagnostic_msgs::msg::DiagnosticStatus::OK;
            break;
        case uavcan_diagnostic_Severity_1_0_NOTICE:
        case uavcan_diagnostic_Severity_1_0_WARNING:
            severity = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            break;
        case uavcan_diagnostic_Severity_1_0_ERROR:
        case uavcan_diagnostic_Severity_1_0_CRITICAL:
        case uavcan_diagnostic_Severity_1_0_ALERT:
            severity = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            break;
        default:
            severity = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    }
    ros_diagnostic.level = severity;

    return ros_diagnostic;
}

}
