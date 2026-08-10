#include "ultrasonic_radar_driver/ultrasonic_radar_node.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace ultrasonic_radar_driver
{

UltrasonicRadarNode::UltrasonicRadarNode(const rclcpp::NodeOptions & options)
: Node("ultrasonic_radar_node", options),
  socket_fd_(-1),
  running_(true)
{
  // 声明参数
  this->declare_parameter<std::string>("can_interface", "vcan3");
  can_interface_ = this->get_parameter("can_interface").as_string();

  // 创建发布者
  pub_ = this->create_publisher<ultrasonic_radar_driver::msg::UltrasonicData>(
    "ultrasonic_radar/data", 10);

  RCLCPP_INFO(this->get_logger(), "超声波雷达驱动节点启动");
  RCLCPP_INFO(this->get_logger(), "CAN接口: %s", can_interface_.c_str());

  // 打开SocketCAN
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "创建CAN socket失败: %s", strerror(errno));
    return;
  }

  // 查找接口索引
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "获取CAN接口索引失败: %s", strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return;
  }

  // 绑定接口
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "绑定CAN socket失败: %s", strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "CAN接口 %s 打开成功", can_interface_.c_str());

  // 启动读取线程
  read_thread_ = std::thread(&UltrasonicRadarNode::canReadThread, this);
}

UltrasonicRadarNode::~UltrasonicRadarNode()
{
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
  RCLCPP_INFO(this->get_logger(), "超声波雷达驱动节点关闭");
}

void UltrasonicRadarNode::canReadThread()
{
  struct can_frame frame;
  fd_set readfds;
  struct timeval tv;

  while (running_ && rclcpp::ok()) {
    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms超时

    int ret = select(socket_fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret > 0 && FD_ISSET(socket_fd_, &readfds)) {
      ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
      if (nbytes > 0) {
        parseCanFrame(frame.can_id, frame.data, frame.can_dlc);
      }
    }
  }
}

void UltrasonicRadarNode::parseCanFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
  // 过滤标准帧ID
  can_id &= CAN_SFF_MASK;

  switch (can_id) {
    case CAN_ID_RESULT:  // 0x18D (397) - 结果/温度
      if (dlc >= 8) {
        // DBC (Motorola @0+):
        //   Byte0-1: Dampling  (16位)
        //   Byte2-3: MeasSlot  (16位)
        //   Byte4:   Dis1
        //   Byte5:   T_OK
        //   Byte6:   T (温度 ℃)
        radar_data_.temperature_ok = (data[5] != 0);
        radar_data_.temperature = static_cast<float>(data[6]);
      }
      break;

    case CAN_ID_RADAR_DIST1:  // 0x3A1 (929) - 后雷达距离1
      if (dlc >= 8) {
        // DBC (Motorola @0+, 每信号8位):
        //   Byte0: RearDis_RRS_TX_RRS_RX  后右泊车自发自收
        //   Byte1: RearDis_RR_TX_RR_RX    后右自发自收
        //   Byte2: RearDis_RR_TX_RRM_RX   后右发后右中收
        //   Byte3: RearDis_RRM_TX_RRM_RX  后右中自发自收
        //   Byte4: RearDis_RRM_TX_RLM_RX  后右中发后左中收
        //   Byte5: RearDis_RRM_TX_RR_RX   后右中发后右收
        //   Byte6: RearDis_RLM_TX_RLM_RX  后左中自发自收
        //   Byte7: RearDis_RLM_TX_RRM_RX  后左中发后右中收
        radar_data_.rear_right_corner = static_cast<float>(data[0]) * DISTANCE_SCALE;
        radar_data_.rear_right       = static_cast<float>(data[1]) * DISTANCE_SCALE;
        radar_data_.rear_right_mid   = static_cast<float>(data[3]) * DISTANCE_SCALE;
        radar_data_.rear_left_mid    = static_cast<float>(data[6]) * DISTANCE_SCALE;
      }
      break;

    case CAN_ID_RADAR_DIST2:  // 0x3A2 (930) - 混合距离
      if (dlc >= 8) {
        // DBC (Motorola @0+, 每信号8位):
        //   Byte0: RearDis_RLM_TX_RL_RX   后左中发后左收
        //   Byte1: RearDis_RL_TX_RL_RX    后左自发自收
        //   Byte2: RearDis_RL_TX_RLM_RX   后左发后左中收
        //   Byte3: RearDis_RLS_TX_RLS_RX  后左泊车自发自收
        //   Byte4: FrontDis_FLM_TX_FL_RX  前左中发前左收
        //   Byte5: FrontDis_FL_TX_FL_RX   前左自发自收
        //   Byte6: FrontDis_FL_TX_FLM_RX  前左发前左中收
        //   Byte7: FrontDis_FLS_TX_FLS_RX 前左泊车自发自收
        radar_data_.rear_left         = static_cast<float>(data[1]) * DISTANCE_SCALE;
        radar_data_.rear_left_corner  = static_cast<float>(data[3]) * DISTANCE_SCALE;
        radar_data_.front_left        = static_cast<float>(data[5]) * DISTANCE_SCALE;
        radar_data_.front_left_corner = static_cast<float>(data[7]) * DISTANCE_SCALE;
      }
      break;

    case CAN_ID_RADAR_DIST3:  // 0x3A3 (931) - 前雷达距离
      if (dlc >= 8) {
        // DBC (Motorola @0+, 每信号8位):
        //   Byte0: FrontDis_FRS_TX_FRS_RX 前右泊车自发自收
        //   Byte1: FrontDis_FR_TX_FR_RX   前右自发自收
        //   Byte2: FrontDis_FR_TX_FRM_RX  前右发前右中收
        //   Byte3: FrontDis_FRM_TX_FRM_RX 前右中自发自收
        //   Byte4: FrontDis_FRM_TX_FLM_RX 前右中发前左中收
        //   Byte5: FrontDis_FRM_TX_FR_RX  前右中发前右收
        //   Byte6: FrontDis_FLM_TX_FLM_RX 前左中自发自收
        //   Byte7: FrontDis_FLM_TX_FRM_RX 前左中发前右中收
        radar_data_.front_right_corner = static_cast<float>(data[0]) * DISTANCE_SCALE;
        radar_data_.front_right        = static_cast<float>(data[1]) * DISTANCE_SCALE;
        radar_data_.front_right_mid    = static_cast<float>(data[3]) * DISTANCE_SCALE;
        radar_data_.front_left_mid     = static_cast<float>(data[6]) * DISTANCE_SCALE;
      }
      break;

    default:
      return;
  }

  publishData();
}

void UltrasonicRadarNode::publishData()
{
  radar_data_.header.stamp = this->now();
  radar_data_.header.frame_id = "ultrasonic_radar";
  pub_->publish(radar_data_);
}

}  // namespace ultrasonic_radar_driver