#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/state_sharing_msg.hpp>
#include <px4_msgs/msg/state_sharing_control.hpp>

#include "state_machine.hpp"
#include "velocity_controller.hpp"

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
  using OffboardControlMode     = px4_msgs::msg::OffboardControlMode;
  using TrajectorySetpoint      = px4_msgs::msg::TrajectorySetpoint;
  using VehicleCommandSrv       = px4_msgs::srv::VehicleCommand;
  using VehicleLocalPosMsg      = px4_msgs::msg::VehicleLocalPosition;
  using StateSharingMsg         = px4_msgs::msg::StateSharingMsg;
  using StateSharingControlMsg  = px4_msgs::msg::StateSharingControl;

public:
  explicit OffboardControl(const rclcpp::NodeOptions &opts =
      rclcpp::NodeOptions());  void requestOffboardMode();
  void arm();
  void disarm();

private:
    enum class State {
    WaitForHeartbeat,
    StartingStateSharing,
    StableStateSharing,
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
    VelocityController    vel_ctrl_;        
    Eigen::Vector3d       current_pos_{};
    bool have_ref_{false};
    double ref_lat_{0.0}, ref_lon_{0.0}, ref_alt_{0.0};   

    /* ───── Parameters ───── */
    double takeoff_alt_;   ///< metres (-Z in NED)
    double tick_hz_;   ///< control loop frequency
    double max_velocity_;  ///< m/s cap on the resulting velocity norm
    double min_altitude_;  ///< metres AMSL—don’t let Z go below this

    /* ───── PX4 identifiers ───── */
    uint8_t ident_{1};           ///< SYS_ID – 1-indexed like MAVLink
    std::string prefix_;
  
    /* ───── ROS interfaces ───── */
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_setpoint_pub_;
    rclcpp::Subscription<VehicleLocalPosMsg>::SharedPtr local_pos_sub_;
    rclcpp::Client<VehicleCommandSrv>::SharedPtr      vehicle_cmd_cli_;
    rclcpp::TimerBase::SharedPtr                      timer_;
    rclcpp::Subscription<StateSharingMsg>::SharedPtr  state_sharing_out_sub_;
    rclcpp::Publisher<StateSharingControlMsg>::SharedPtr state_sharing_ctrl_pub_;

    /* ───── Service-reply bookkeeping ───── */
    void startStateSharing();
    bool state_sharing_active_{false};
    uint8_t service_result_{0};
    bool    service_done_{false};

    /* ───── Helper API ───── */
    void sendVehicleCommand(uint16_t cmd, float p1 = 0.f, float p2 = 0.f);
    void publishOffboardCtrlMode(bool pos_ctrl, bool vel_ctrl);
    void publishPositionSetpoint(double x, double y, double z, double yaw);
    void publishVelocitySetpoint(double vx, double vy, double vz, double yaw_rate);
    void publishCurrentSetpoint();

    /* ───── Callbacks ───── */
    void onCmdResult(rclcpp::Client<VehicleCommandSrv>::SharedFuture);
    void onTimer();
    void onLocalPos(VehicleLocalPosMsg::SharedPtr msg);
    void onStateSharing(const std::shared_ptr<StateSharingMsg> msg);


    /* ───── Non-copyable ───── */
    OffboardControl(const OffboardControl &)            = delete;
    OffboardControl &operator=(const OffboardControl &) = delete;
};

}  // namespace state_sharing_demo
