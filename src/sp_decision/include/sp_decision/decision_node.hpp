#ifndef SP_DECISION__DECISION_NODE_HPP
#define SP_DECISION__DECISION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <sp_interfaces/msg/gimbal_command.hpp>
#include <sp_interfaces/msg/gimbal_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cstdint>

namespace sp_decision
{
class DecisionNode : public rclcpp::Node
{
public:
  explicit DecisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  void stateCallback(const sp_interfaces::msg::GimbalState::SharedPtr msg);
  void bulletSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void publishCommand(double yaw, double pitch, bool control, bool fire);

  // 订阅
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<sp_interfaces::msg::GimbalState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bullet_speed_sub_;

  // 发布
  rclcpp::Publisher<sp_interfaces::msg::GimbalCommand>::SharedPtr command_pub_;

  // 最新状态值
  uint8_t current_mode_ = 0;  // 0:idle 1:auto_aim 2:small_buff 3:big_buff
  double bullet_speed_ = 15.0;
  double current_yaw_ = 0.0;
  double current_pitch_ = 0.0;

  // 可调参数
  double total_delay_ = 0.03;
  double fire_tolerance_ = 0.03;
  double yaw_offset_ = 0.0;
  double pitch_offset_ = 0.0;
};

}  // namespace sp_decision

#endif  // SP_DECISION__DECISION_NODE_HPP
