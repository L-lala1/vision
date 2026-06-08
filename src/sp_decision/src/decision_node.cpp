#include "sp_decision/decision_node.hpp"
#include "sp_decision/trajectory.hpp"
#include "sp_decision/math_tools.hpp"

#include <cmath>

namespace sp_decision
{

DecisionNode::DecisionNode(const rclcpp::NodeOptions & options)
: Node("decision_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DecisionNode");

  // 从参数读取
  total_delay_ = this->declare_parameter("total_delay", 0.03);
  fire_tolerance_ = this->declare_parameter("fire_tolerance", 0.03);
  yaw_offset_ = this->declare_parameter("yaw_offset", 0.0);
  pitch_offset_ = this->declare_parameter("pitch_offset", 0.0);

  // 订阅 tracker 输出
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&DecisionNode::targetCallback, this, std::placeholders::_1));

  // 订阅云台状态
  state_sub_ = this->create_subscription<sp_interfaces::msg::GimbalState>(
    "/gimbal/state", 10,
    std::bind(&DecisionNode::stateCallback, this, std::placeholders::_1));

  // 订阅子弹速度（作为辅助，如果 MCU 不发则用默认值）
  bullet_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/gimbal/bullet_speed", 10,
    std::bind(&DecisionNode::bulletSpeedCallback, this, std::placeholders::_1));

  // 发布云台指令
  command_pub_ = this->create_publisher<sp_interfaces::msg::GimbalCommand>(
    "/gimbal/command", 10);
}

void DecisionNode::stateCallback(const sp_interfaces::msg::GimbalState::SharedPtr msg)
{
  current_mode_ = msg->mode;
  current_yaw_ = msg->yaw;
  current_pitch_ = msg->pitch;
}

void DecisionNode::bulletSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  bullet_speed_ = msg->data;
}

void DecisionNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  if (!msg->tracking) {
    publishCommand(0.0, 0.0, false, false);
    return;
  }

  // Compute actual system latency from message timestamp (dynamic, not fixed 30ms)
  double measured_latency = (this->now() - msg->header.stamp).seconds();
  // Use measured latency but clamp to [total_delay_, 0.15] to avoid extreme values
  double effective_delay = std::max(total_delay_, std::min(measured_latency, 0.15));

  // 1. 弹道解算
  double d = std::hypot(msg->position.x, msg->position.y);
  double h = msg->position.z;
  tools::Trajectory traj(bullet_speed_, d, h);

  if (traj.unsolvable) {
    publishCommand(0.0, 0.0, false, false);
    return;
  }

  // 2. 用弹道飞行时间 + 实测延迟 重新预测未来位置
  double future_t = traj.fly_time + effective_delay;
  double fx = msg->position.x + msg->velocity.x * future_t;
  double fy = msg->position.y + msg->velocity.y * future_t;
  double fz = msg->position.z + msg->velocity.z * future_t;

  // 用未来高度重新算弹道（迭代一次更精准）
  tools::Trajectory traj2(bullet_speed_, std::hypot(fx, fy), fz);
  if (!traj2.unsolvable) {
    traj = traj2;
    // Re-compute future_t with corrected fly_time for better accuracy
    future_t = traj.fly_time + effective_delay;
    fx = msg->position.x + msg->velocity.x * future_t;
    fy = msg->position.y + msg->velocity.y * future_t;
    fz = msg->position.z + msg->velocity.z * future_t;
  }

  // 3. 旋转补偿 + 计算瞄准角度
  // 弹丸飞行期间目标在转，小陀螺转速 >180°/s 时 0.3s 飞行可旋转 >54°
  // 必须瞄准装甲板将转到的位置，而非当前位置
  double future_yaw = msg->yaw + msg->v_yaw * future_t;
  double r = msg->radius_1;
  // 装甲板位置 = 中心 - r*(cos(yaw), sin(yaw))
  double armor_x = fx - r * std::cos(future_yaw);
  double armor_y = fy - r * std::sin(future_yaw);

  double aim_yaw = std::atan2(armor_y, armor_x) + yaw_offset_;
  double aim_pitch = -(traj.pitch + pitch_offset_);

  // 4. 开火判断
  bool control = (current_mode_ == 1);  // 只有自瞄模式才控制云台
  bool fire = false;
  if (control) {
    double yaw_err = std::abs(tools::limit_rad(aim_yaw - current_yaw_));
    double pitch_err = std::abs(aim_pitch - current_pitch_);
    fire = (yaw_err < fire_tolerance_ && pitch_err < fire_tolerance_);
  }

  // 5. 发布
  publishCommand(aim_yaw, aim_pitch, control, fire);
}

void DecisionNode::publishCommand(double yaw, double pitch, bool control, bool fire)
{
  auto cmd = sp_interfaces::msg::GimbalCommand();
  cmd.control = control;
  cmd.fire = fire;
  cmd.yaw = yaw;
  cmd.pitch = pitch;
  cmd.yaw_vel = 0.0;
  cmd.yaw_acc = 0.0;
  cmd.pitch_vel = 0.0;
  cmd.pitch_acc = 0.0;
  command_pub_->publish(cmd);
}

}  // namespace sp_decision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sp_decision::DecisionNode)
