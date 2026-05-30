#ifndef SP_GIMBAL__PACKET_HPP
#define SP_GIMBAL__PACKET_HPP

#include <cstdint>

namespace sp_gimbal
{
// MCU → 视觉: 云台状态 + IMU四元数 + 子弹数据
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

// 视觉 → MCU: 云台控制指令 (位置+速度+加速度前馈)
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) <= 64);

}  // namespace sp_gimbal

#endif  // SP_GIMBAL__PACKET_HPP
