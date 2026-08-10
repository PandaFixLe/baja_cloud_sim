#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class ChcnavMiniTest : public rclcpp::Node
{
public:
    ChcnavMiniTest() : Node("chcnav_mini_test")
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/chcnav/devimu", 10);
        
        if (!open_can("vcan2")) {
            RCLCPP_ERROR(this->get_logger(), "打开 vcan2 失败！请确保 sudo ip link set vcan2 up 已执行。");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "?? 极简探测器启动！等待读取 vcan2...");

        read_thread_ = std::thread(&ChcnavMiniTest::read_loop, this);
    }

    ~ChcnavMiniTest()
    {
        if (read_thread_.joinable())
            read_thread_.join();
        if (sockfd_ >= 0) close(sockfd_);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    int sockfd_ = -1;
    std::thread read_thread_;

    bool open_can(const std::string& ifname)
    {
        sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sockfd_ < 0) return false;

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, ifname.c_str());
        if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) { close(sockfd_); return false; }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(sockfd_); return false; }
        return true;
    }

    void read_loop()
    {
        struct can_frame frame;
        while (rclcpp::ok()) {
            ssize_t n = read(sockfd_, &frame, sizeof(frame));
            if (n < 0) continue;
            if (n != sizeof(frame)) continue;
            
            // 不解析错误帧和远程帧
            if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) continue;

            uint32_t id = frame.can_id & CAN_SFF_MASK;
            
            // 【核心日志】只要收到任何报文，立刻打印！
            RCLCPP_INFO(this->get_logger(), "??【探测成功】收到 CAN 帧，ID = 0x%03X", id);

            // 只要收到 0x321（角速度），我们就强制发一个假数据，证明话题没死
            if (id == 0x321) {
                sensor_msgs::msg::Imu msg;
                msg.header.stamp = this->now();
                msg.angular_velocity.x = 0.1;
                msg.angular_velocity.y = 0.2;
                msg.angular_velocity.z = 0.3;
                imu_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "? 已强制向 /chcnav/devimu 发布测试数据！");
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChcnavMiniTest>());
    rclcpp::shutdown();
    return 0;
}