#include "sp_gimbal/gimbal_node.hpp"

// 原来的 CRC 和 logger 替换：
#include "sp_gimbal/crc.hpp"          // tools/crc.hpp → 路径变了
// 不再 include tools/logger.hpp，ROS2 自带日志

// 新增：
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <memory>

namespace sp_gimbal
{

GimbalNode::GimbalNode(const rclcpp::NodeOptions & options)
: Node("gimbal_node", options)    // 节点名
{
  RCLCPP_INFO(this->get_logger(), "Starting GimbalNode");

  // 从参数获取串口配置（默认值兜底）
  auto com_port = this->declare_parameter("com_port", "/dev/gimbal");

  // 打开串口（和原来一样）
  serial_.setPort(com_port);
  try {
    serial_.open();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // 启动接收线程（和原来一样）
  thread_ = std::thread(&GimbalNode::read_thread, this);

  // ★ 新增：创建 tf 广播、topic 发布、订阅
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  bullet_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gimbal/bullet_speed", 10);
  gimbal_state_pub_ = this->create_publisher<sp_interfaces::msg::GimbalState>("/gimbal/state", 10);
  command_sub_ = this->create_subscription<sp_interfaces::msg::GimbalCommand>(
    "/gimbal/command", rclcpp::SensorDataQoS(),
    std::bind(&GimbalNode::commandCallback, this, std::placeholders::_1));
}

GimbalNode::~GimbalNode()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}
bool GimbalNode::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
        RCLCPP_WARN(this->get_logger(), "Failed to read serial: %s", e.what());
    return false;
  }
}
void GimbalNode::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    RCLCPP_WARN(this->get_logger(),"[Gimbal] Reconnecting serial, attempt %d/%d...", i + 1, max_retry_count);
    try {
      serial_.close();
      
    } catch (...) {
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    try {
      serial_.open();  // 尝试重新打开

      RCLCPP_INFO(this->get_logger(),"[GimbalNode] Reconnected serial successfully.");
      return;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(),"[GimbalNode] Reconnect failed: %s", e.what() );
    }
     

  }
  RCLCPP_ERROR(this->get_logger(), "[GimbalNode] Reconnect failed after all retries.");

}

void GimbalNode::read_thread()
{
  RCLCPP_INFO(this->get_logger(),"[GimbalNode] read_thread started." )//tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      RCLCPP_WARN(this->get_logger(),"[GimbalNode] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      RCLCPP_DEBUG(this->get_logger(),"[GimbalNode] CRC16 check failed.");
      continue;
    }


    error_count = 0;
    
// ── 1. 发布 tf（odom → gimbal_link） ──
geometry_msgs::msg::TransformStamped t;
t.header.stamp = this->now();
t.header.frame_id = "odom";
t.child_frame_id = "gimbal_link";
// 四元数 wxyz → xyzw（注意顺序转换！）
tf2::Quaternion q(rx_data_.q[1], rx_data_.q[2], rx_data_.q[3], rx_data_.q[0]);
t.transform.rotation = tf2::toMsg(q);
tf_broadcaster_->sendTransform(t);

// ── 2. 发布 bullet_speed ──
auto speed_msg = std_msgs::msg::Float32();

speed_msg.data = rx_data_.bullet_speed;
bullet_speed_pub_->publish(speed_msg);

// ── 3. 发布 gimbal_state ──
auto state_msg = sp_interfaces::msg::GimbalState();
state_msg.mode = rx_data_.mode;

state_msg.yaw = rx_data_.yaw;
state_msg.yaw_vel = rx_data_.yaw_vel;
state_msg.pitch = rx_data_.pitch;
state_msg.pitch_vel = rx_data_.pitch_vel;
state_msg.bullet_speed = rx_data_.bullet_speed;
state_msg.bullet_count = rx_data_.bullet_count;
gimbal_state_pub_->publish(state_msg);


   
  }

  RCLCPP_INFO(this->get_logger(),"[GimbalNode] read_thread stopped.");
}
void GimbalNode::commandCallback(const sp_interfaces::msg::GimbalCommand::SharedPtr msg)
{
  VisionToGimbal packet;
  packet.head[0] = 'S';
  packet.head[1] = 'P';
  packet.mode = msg->fire ? 2 : (msg->control ? 1 : 0);
  packet.yaw       = msg->yaw;
  packet.yaw_vel   = msg->yaw_vel;
  packet.yaw_acc   = msg->yaw_acc;
  packet.pitch     = msg->pitch;
  packet.pitch_vel = msg->pitch_vel;
  packet.pitch_acc = msg->pitch_acc;
  packet.crc16 = tools::get_crc16(
    reinterpret_cast<const uint8_t *>(&packet),
    sizeof(packet) - sizeof(packet.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to write serial: %s", e.what());
  }
}



}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sp_gimbal::GimbalNode)



    