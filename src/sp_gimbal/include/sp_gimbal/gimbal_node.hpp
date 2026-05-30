#ifndef SP_GIMBAL__GIMBAL_NODE_HPP
#define SP_GIMBAL__GIMBAL_NODE_HPP


#include <rclcpp/rclcpp.hpp>                    // Node 基类
#include <tf2_ros/transform_broadcaster.h>      // 发布 tf
#include <std_msgs/msg/float32.hpp>             // bullet_speed topic
#include <sp_interfaces/msg/gimbal_command.hpp>  // 订阅的命令
#include <sp_interfaces/msg/gimbal_state.hpp>    // 发布的云台状态

#include "sp_gimbal/packet.hpp"   //
#include "serial/serial.h"
#include <cstdint>    // uint8_t, uint16_t
#include <thread>     // std::thread
#include <atomic>     // std::atomic<bool>
//#include <chrono>     // std::chrono::steady_clock
//#include <mutex> // std::mutex（如果有线程安全需求）
#include <memory>

namespace sp_gimbal
{
class GimbalNode : public rclcpp::Node   // 关键：继承 ROS2 的 Node
{
public:
  explicit GimbalNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GimbalNode();

private:
  // ====== 收发线程 ======
  void read_thread();    // 和原来 Gimbal::read_thread() 一模一样
  bool read(uint8_t * buffer, size_t size);  // 从串口读 n 字节
  void reconnect();      // 串口重连

  // ====== 订阅回调（新增，原来没有） ======
  void commandCallback(const sp_interfaces::msg::GimbalCommand::SharedPtr msg);

  // ====== 串口 ======
  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_{false};

  // ====== 接收数据 ======
  GimbalToVision rx_data_;
  // 四元数队列（给外部 slerp 查询用，暂时可跳过）
  // tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, TimePoint>> queue_;

  // ====== ROS2 发布和订阅 ======
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bullet_speed_pub_;
  rclcpp::Publisher<sp_interfaces::msg::GimbalState>::SharedPtr gimbal_state_pub_;
  rclcpp::Subscription<sp_interfaces::msg::GimbalCommand>::SharedPtr command_sub_;
};

}  // namespace sp_gimbal



