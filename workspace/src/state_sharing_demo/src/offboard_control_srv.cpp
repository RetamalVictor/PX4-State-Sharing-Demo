#include "offboard_control_srv.hpp"

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace state_sharing_demo {

/*──────────────────────────────────────────────────────────────────────────*/
/*                             Constructor                                  */
/*──────────────────────────────────────────────────────────────────────────*/

OffboardControl::OffboardControl(const rclcpp::NodeOptions &opts)
    : rclcpp::Node("offboard_control", opts)
{
    /* ─── Declare & get parameters ─────────────────────────────────────────*/
    takeoff_alt_ = declare_parameter<double>("takeoff_alt", 5.0);   // [m]
    tick_hz_     = declare_parameter<double>("tick_hz",     10.0);  // [Hz]

    /* ─── Derive namespace & MAVLink SYS_ID ───────────────────────────────*/
    std::string ns = get_namespace();  // e.g. "/px4_2"
    if (ns.rfind("/px4_", 0) == 0) {
    ident_ = static_cast<uint8_t>(std::stoi(ns.substr(5)));  // zero‑indexed
    } else {
    RCLCPP_WARN(get_logger(),
                "Namespace '%s' does not follow /px4_N; default ident = 1",
                ns.c_str());
    ident_ = 1;
    }
    std::string prefix = "/" + ns.substr(1) + "/fmu/";  // "/px4_N/fmu/"

    /* ─── Publishers / client / subscriber ────────────────────────────────*/
    offboard_ctrl_mode_pub_ = create_publisher<OffboardControlMode>(
        prefix + "in/offboard_control_mode", 10);

    traj_setpoint_pub_ = create_publisher<TrajectorySetpoint>(
        prefix + "in/trajectory_setpoint", 10);

    vehicle_cmd_cli_ = create_client<VehicleCommandSrv>(prefix + "vehicle_command");

    local_pos_sub_ = create_subscription<VehicleLocalPosMsg>(
        prefix + "out/vehicle_local_position", 10,
        rclcpp::SensorDataQoS(),  
        std::bind(&OffboardControl::onLocalPos, this, std::placeholders::_1));

    /* ─── Wait for vehicle_command service ────────────────────────────────*/
    using namespace std::chrono_literals;
    while (!vehicle_cmd_cli_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(get_logger(), "Waiting for vehicle_command service …");
    }

    /* ─── Create main timer ───────────────────────────────────────────────*/
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1 / tick_hz_),
        std::bind(&OffboardControl::onTimer, this));
    // timer_ = create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

    RCLCPP_INFO(get_logger(),
                "OffboardControl: ns=%s SYS_ID=%u takeoff_alt=%.1f m @ %.0f Hz",
                ns.c_str(), ident_ + 1, takeoff_alt_, tick_hz_);
}

/*──────────────────────────────────────────────────────────────────────────*/
/*                         Helper publishing API                            */
/*──────────────────────────────────────────────────────────────────────────*/

void OffboardControl::publishOffboardCtrlMode(bool pos_ctrl, bool vel_ctrl)
{
  OffboardControlMode msg{};
  msg.position     = pos_ctrl;
  msg.velocity     = vel_ctrl;
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;
  msg.timestamp    = get_clock()->now().nanoseconds() / 1000ULL;
  offboard_ctrl_mode_pub_->publish(msg);
}

void OffboardControl::publishPositionSetpoint(double x, double y, double z, double yaw)
{
  TrajectorySetpoint sp{};
  sp.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
  sp.yaw      = static_cast<float>(yaw);
  sp.timestamp = get_clock()->now().nanoseconds() / 1000ULL;
  traj_setpoint_pub_->publish(sp);
}

void OffboardControl::publishVelocitySetpoint(double vx, double vy, double vz, double yaw_rate)
{
  TrajectorySetpoint sp{};
  sp.velocity     = {static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz)};
  sp.yaw          = 0.0f;
  sp.yawspeed     = static_cast<float>(yaw_rate);
  sp.timestamp    = get_clock()->now().nanoseconds() / 1000ULL;
  traj_setpoint_pub_->publish(sp);
}

/*──────────────────────────────────────────────────────────────────────────*/
/*                     Vehicle‑command wrapper                              */
/*──────────────────────────────────────────────────────────────────────────*/

void OffboardControl::sendVehicleCommand(uint16_t cmd, float p1, float p2)
{
  auto request = std::make_shared<VehicleCommandSrv::Request>();

  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = p1;
  msg.param2 = p2;
  msg.command = cmd;
  msg.target_system    = ident_ + 1;  // PX4 expects 1‑indexed
  msg.target_component = 1;
  msg.source_system    = ident_ + 1;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = get_clock()->now().nanoseconds() / 1000ULL;

  request->request = msg;
  service_done_    = false;

  vehicle_cmd_cli_->async_send_request(
      request,
      std::bind(&OffboardControl::onCmdResult, this, std::placeholders::_1));
}

/*──────────────────────────────────────────────────────────────────────────*/
/*                        Public command helpers                            */
/*──────────────────────────────────────────────────────────────────────────*/

void OffboardControl::requestOffboardMode()
{
  RCLCPP_INFO(get_logger(), "Requesting Off‑board mode");
  sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                     /*param1=*/1.0f, /*param2=*/6.0f);
}

void OffboardControl::arm()
{
  RCLCPP_INFO(get_logger(), "Arming");
  sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                     1.0f);
}

void OffboardControl::disarm()
{
  RCLCPP_INFO(get_logger(), "Disarming");
  sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                     0.0f);
}

/*──────────────────────────────────────────────────────────────────────────*/
/*                           Callbacks                                      */
/*──────────────────────────────────────────────────────────────────────────*/

void OffboardControl::onCmdResult(
    rclcpp::Client<VehicleCommandSrv>::SharedFuture future)
{
  auto status = future.wait_for(100ms);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "VehicleCommand service timed‑out");
    return;
  }

  auto reply = future.get()->reply;
  service_result_ = reply.result;
  service_done_   = true;

  static const char *table[] = {
      "ACCEPTED", "TEMP_REJ", "DENIED", "UNSUPPORTED",
      "FAILED",   "IN_PROGRESS", "CANCELLED"};
  uint8_t idx = std::min<uint8_t>(reply.result, 6);
  RCLCPP_INFO(get_logger(), "Cmd result: %s", table[idx]);
}

/* Store altitude (‑z in NED) */
static constexpr double NAN_ALT = std::numeric_limits<double>::quiet_NaN();
static double current_alt_ = NAN_ALT;

void OffboardControl::onLocalPos(VehicleLocalPosMsg::SharedPtr msg)
{
  current_alt_ = -static_cast<double>(msg->z);  // NED z is down ⇒ invert
}

/*──────────────────────────────────────────────────────────────────────────*/
/*                             Main timer                                   */
/*──────────────────────────────────────────────────────────────────────────*/
void OffboardControl::publishCurrentSetpoint()
{
  using enum State;
  switch (fsm_.state()) {
    case State::VelocityControl:
      publishOffboardCtrlMode(false, true);
      publishVelocitySetpoint(1.0, 0.0, 0.0, 0.0);
      break;

    default: /* WaitForHeartbeat, RequestOffboard, StabiliseOffboard, ArmRequested, Takeoff, Hover, Landing */
      publishOffboardCtrlMode(true, false);
      publishPositionSetpoint(0.0, 0.0, -takeoff_alt_, 0.0);
      break;
  }
}

void OffboardControl::onTimer()
{
  using enum State;
  publishCurrentSetpoint();

  switch (fsm_.state()) {
    case State::WaitForHeartbeat:
      /* In real systems you would wait for an FMU heartbeat topic.   */
      /* For the example we immediately request Off‑board mode.       */
      requestOffboardMode();
      fsm_.transit(RequestOffboard);
      break;

    case State::RequestOffboard:
      if (service_done_ && service_result_ == 0) {
        fsm_.transit(StabiliseOffboard);
      }
      break;

    case State::StabiliseOffboard:
      if (fsm_.since(1.0)) {   // 1 s stable
        arm();
        fsm_.transit(ArmRequested);
      }
      break;

    case State::ArmRequested:
      if (service_done_ && service_result_ == 0) {
        fsm_.transit(Takeoff);
      }
      break;

    case State::Takeoff:
      // /* Position control until altitude reached */
      // publishOffboardCtrlMode(true, false);
      // publishPositionSetpoint(0.0, 0.0, -takeoff_alt_, 0.0);

      if (!std::isnan(current_alt_) && current_alt_ >= takeoff_alt_ - 0.2) {
        fsm_.transit(Hover);
      }
      break;

    case State::Hover:
      // publishOffboardCtrlMode(true, false);
      // publishPositionSetpoint(0.0, 0.0, -takeoff_alt_, 0.0);
      if (fsm_.since(2.0)) {   // hover 2 s
        fsm_.transit(VelocityControl);
      }
      break;

    case State::VelocityControl:
      // publishOffboardCtrlMode(false, true);
      // publishVelocitySetpoint(1.0, 0.0, 0.0, 0.0);  // fly forward 1 m/s
      break;

    case Landing:
      /* Not implemented */
      break;
  }
}

}  // namespace state_sharing_demo

