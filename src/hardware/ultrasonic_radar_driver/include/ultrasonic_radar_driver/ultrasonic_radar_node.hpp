#ifndef ULTRASONIC_RADAR_NODE_HPP_
#define ULTRASONIC_RADAR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ultrasonic_radar_driver/msg/ultrasonic_data.hpp>
#include <string>
#include <thread>
#include <atomic>

// CAN ID 定义
constexpr uint32_t CAN_ID_RESULT        = 0x18D;  // 397 - 结果/温度
constexpr uint32_t CAN_ID_RADAR_DIST1   = 0x3A1;  // 929 - 后雷达距离1
constexpr uint32_t CAN_ID_RADAR_DIST2   = 0x3A2;  // 930 - 混合距离
constexpr uint32_t CAN_ID_RADAR_DIST3   = 0x3A3;  // 931 - 前雷达距离

// 距离精度系数 (每个字节代表2cm)
constexpr float DISTANCE_SCALE = 2.0f;

namespace ultrasonic_radar_driver
{

class UltrasonicRadarNode : public rclcpp::Node
{
public:
  explicit UltrasonicRadarNode(const rclcpp::NodeOptions & options);
  ~UltrasonicRadarNode();

private:
  void canReadThread();
  void parseCanFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
  void publishData();

  // SocketCAN
  int socket_fd_;
  std::string can_interface_;

  // 读取线程
  std::thread read_thread_;
  std::atomic<bool> running_;

  // 发布者
  rclcpp::Publisher<ultrasonic_radar_driver::msg::UltrasonicData>::SharedPtr pub_;

  // 缓存数据
  ultrasonic_radar_driver::msg::UltrasonicData radar_data_;
};

}  // namespace ultrasonic_radar_driver

#endif  // ULTRASONIC_RADAR_NODE_HPP_