#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

#include "state_machine.hpp"

namespace state_sharing_demo {

/**
 * @brief Single-UAV off-board controller.
 *
 * Flight phases:
 *  1. Wait for FMU heartbeat.
 *  2. Request off-board mode.
 *  3. Stabilise for a short period.
 *  4. Arm.
 *  5. *Takeoff*   (position control up to `takeoff_alt` metres)
 *  6. *Hover*     (hold position)
 *  7. *VelocityControl* (switch to velocity set-points)
 *
 * Two parameters make the behaviour field-tunable:
 *  * `takeoff_alt`  — positive altitude (metres) to reach before switching
 *  * `tick_hz`      — main control-loop frequency
 */

class OffboardControl final : public rclcpp::Node
{
  using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
  using TrajectorySetpoint  = px4_msgs::msg::TrajectorySetpoint;
  using VehicleCommandSrv   = px4_msgs::srv::VehicleCommand;
  using VehicleLocalPosMsg  = px4_msgs::msg::VehicleLocalPosition;

public:
  explicit OffboardControl(const rclcpp::NodeOptions &opts = {});
  void requestOffboardMode();
  void arm();
  void disarm();

private:
    /* ───── Finite-state definition ───── */
    enum class State {
    WaitForHeartbeat,
    RequestOffboard,
    StabiliseOffboard,
    ArmRequested,
    Armed,
    Takeoff,
    Hover,
    VelocityControl,
    Landing
    };

    StateMachine<State> fsm_{this, State::WaitForHeartbeat};

    /* ───── Parameters ───── */
    double takeoff_alt_;   ///< metres (-Z in NED)
    double tick_hz_;   ///< control loop frequency

    /* ───── PX4 identifiers ───── */
    uint8_t ident_{1};           ///< SYS_ID – 1-indexed like MAVLink

    /* ───── ROS interfaces ───── */
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_setpoint_pub_;
    rclcpp::Client<VehicleCommandSrv>::SharedPtr      vehicle_cmd_cli_;
    rclcpp::TimerBase::SharedPtr                      timer_;

    /* ───── Service-reply bookkeeping ───── */
    uint8_t service_result_{0};
    bool    service_done_{false};

    /* ───── Helper API ───── */
    void sendVehicleCommand(uint16_t cmd, float p1 = 0.f, float p2 = 0.f);
    void publishOffboardCtrlMode(bool pos_ctrl, bool vel_ctrl);
    void publishPositionSetpoint(double x, double y, double z, double yaw);
    void publishVelocitySetpoint(double vx, double vy, double vz, double yaw_rate);


    /* ───── Callbacks ───── */
    void onCmdResult(rclcpp::Client<VehicleCommandSrv>::SharedFuture);
    void onTimer();

    /* ───── Non-copyable ───── */
    OffboardControl(const OffboardControl &)            = delete;
    OffboardControl &operator=(const OffboardControl &) = delete;
};

}  // namespace state_sharing_demo
