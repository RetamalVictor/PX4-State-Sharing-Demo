#pragma once
#include <rclcpp/rclcpp.hpp>
#include <type_traits>

/**
 * @brief Tiny helper class that wraps an enum-based FSM.
 *
 * Features:
 *  * Logs every state transition.
 *  * Tracks how long the node has been in the current state.
 *  * Provides simple convenience utilities (`elapsed()`, `since()`).
 *
 * @tparam Enum  An `enum class` defining the controller states.
 */
template<typename Enum>
class StateMachine
{
  static_assert(std::is_enum_v<Enum>, "StateMachine requires an enum type");

public:
  StateMachine(rclcpp::Node *node, Enum initial) :
      node_{node}, state_{initial}, entered_{node->now()} {}

  /// Current state
  Enum state() const noexcept { return state_; }

  /// Time when the current state was entered
  rclcpp::Time entered() const noexcept { return entered_; }

  /// Duration spent in the current state
  rclcpp::Duration elapsed() const noexcept { return node_->now() - entered_; }

  /// Convenience: true if elapsed() > @p seconds
  bool since(double seconds) const noexcept
  {
    return elapsed() > rclcpp::Duration::from_seconds(seconds);
  }

  /// Transition to @p next (logs the change and resets timer)
  void transit(Enum next)
  {
    if (next == state_) { return; }
    RCLCPP_INFO(node_->get_logger(),
                "FSM: %d -> %d",
                static_cast<int>(state_), static_cast<int>(next));
    state_   = next;
    entered_ = node_->now();
  }

private:
  rclcpp::Node *node_;
  Enum          state_;
  rclcpp::Time  entered_;
};
