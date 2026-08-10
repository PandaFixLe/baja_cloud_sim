#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <cerrno>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "radar_can_parser/radar_parser.h"

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("radar_debug_node"), stop_flag_(false) {
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/radar/point_cloud", 10);

        if (!connect_can()) {
            RCLCPP_ERROR(this->get_logger(), "无法连接 can0，请检查配置");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "调试模式启动：将打印详细数据并发布点云");
        rx_thread_ = std::thread(&RadarNode::receive_thread_loop, this);
        process_thread_ = std::thread(&RadarNode::process_thread_loop, this);
    }

    ~RadarNode() {
        stop_flag_ = true;
        cv_.notify_all();
        if (rx_thread_.joinable()) rx_thread_.join();
        if (process_thread_.joinable()) process_thread_.join();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    RadarDataParser parser_;
    int can_socket_;

    std::thread rx_thread_;
    std::thread process_thread_;
    std::atomic<bool> stop_flag_;
    std::mutex mtx_;
    std::condition_variable cv_;

    std::vector<CanData> buffer_ping_;
    std::vector<CanData> buffer_pong_;
    std::vector<CanData> process_buffer_;
    uint8_t receive_data_flag_ = 0;

    bool connect_can() {
        struct sockaddr_can addr;
        struct ifreq ifr;
        
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "创建 socket 失败: %s", strerror(errno));
            return false;
        }

        int flags = fcntl(can_socket_, F_GETFL, 0);
        if (flags < 0 || fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
            RCLCPP_ERROR(this->get_logger(), "设置 socket 非阻塞模式失败");
            return false;
        }

        int enable_fd = 1;
        if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "设置 CAN FD 标志位失败: %s", strerror(errno));
            return false;
        }

        strcpy(ifr.ifr_name, "can0");
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "can0 接口不存在: %s", strerror(errno));
            return false;
        }
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "bind 失败: %s", strerror(errno));
            return false;
        }
        return true;
    }

    void receive_thread_loop() {
        struct canfd_frame raw;
        CanData canData;
        std::vector<CanData>* current_buffer = &buffer_ping_;

        while (!stop_flag_) {
            int nbytes = read(can_socket_, &raw, sizeof(raw));
            
            if (nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "read 错误: %s", strerror(errno));
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
            }

            if (nbytes == 0) continue;

            canData.ID = raw.can_id & 0x7FF;
            canData.DataLenth = raw.len;
            canData.Channel = 0;
            canData.timeStamp = 0.0;
            for (int i = 0; i < canData.DataLenth && i < 64; ++i)
                canData.Data[i] = raw.data[i];

            // 仅处理后向雷达 (RR 和 RC)
            uint8_t radar_idx = 255;
            if (canData.ID >= 0x600 && canData.ID <= 0x619) radar_idx = RightRearRadar;
            else if (canData.ID >= 0x640 && canData.ID <= 0x659) radar_idx = RearCenterRadar;
            else continue;

            bool is_frame_boundary = ((canData.ID & 0x0F) == 0x00);

            std::unique_lock<std::mutex> lock(mtx_);
            if (is_frame_boundary) {
                if (receive_data_flag_ == 0) {
                    process_buffer_ = std::move(*current_buffer);
                    current_buffer = &buffer_pong_;
                    receive_data_flag_ = 1;
                } else {
                    process_buffer_ = std::move(*current_buffer);
                    current_buffer = &buffer_ping_;
                    receive_data_flag_ = 0;
                }
                lock.unlock();
                cv_.notify_one();
            } else {
                current_buffer->push_back(canData);
                lock.unlock();
            }
        }
    }

    void process_thread_loop() {
        while (!stop_flag_) {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [this]() { return !process_buffer_.empty() || stop_flag_; });
            if (stop_flag_ && process_buffer_.empty()) break;

            auto batch = std::move(process_buffer_);
            lock.unlock();

            for (const auto& can : batch) {
                uint8_t radar_idx = 255;
                if (can.ID >= 0x600 && can.ID <= 0x619) radar_idx = RightRearRadar;
                else if (can.ID >= 0x640 && can.ID <= 0x659) radar_idx = RearCenterRadar;
                if (radar_idx == 255) continue;
                parser_.processCanData(can, radar_idx);
            }

            // 打印调试信息
            print_debug_data();
            publish_data();
        }
    }

    void print_debug_data() {
        // 打印 RR 点云簇的 Range 和 Angle (只打印前几个)
        for (int i = 0; i < parser_.radarPack.PeakFarCount[RightRearRadar] && i < 8; ++i) {
            const auto& pt = parser_.radarPack.PeakFar[RightRearRadar][i];
            RCLCPP_INFO(this->get_logger(), "RR簇 %d: Range=%.2f m, Angle=%.2f rad", i, pt.Range, pt.Angle);
        }
        // 打印 RC 目标
        for (int i = 0; i < parser_.radarPack.ObjectNum[RearCenterRadar]; ++i) {
            const auto& obj = parser_.radarPack.ObjectList[RearCenterRadar][i];
            RCLCPP_INFO(this->get_logger(), "RC目标 %d: DistLong=%.2f m, DistLat=%.2f m", i, obj.DistLong, obj.DistLat);
        }
    }

    void publish_data() {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        const float MAX_RANGE = 50.0f; 
        const float MIN_RANGE = 0.2f;

        // RR
        for (int i = 0; i < parser_.radarPack.PeakFarCount[RightRearRadar]; ++i) {
            const auto& pt = parser_.radarPack.PeakFar[RightRearRadar][i];
            if (pt.Range < MIN_RANGE || pt.Range > MAX_RANGE) continue;
            pcl::PointXYZRGB p;
            p.x = pt.Dx;
            p.y = pt.Dy;
            p.z = pt.Dh;
            p.r = 255; p.g = 255; p.b = 255;
            cloud.points.push_back(p);
        }

        // RC
        for (int i = 0; i < parser_.radarPack.ObjectNum[RearCenterRadar]; ++i) {
            const auto& obj = parser_.radarPack.ObjectList[RearCenterRadar][i];
            float dist = std::sqrt(obj.DistLong*obj.DistLong + obj.DistLat*obj.DistLat);
            if (dist < MIN_RANGE || dist > MAX_RANGE) continue;
            pcl::PointXYZRGB p;
            p.x = obj.DistLong;
            p.y = obj.DistLat;
            p.z = obj.DistHigh;
            p.r = 0; p.g = 255; p.b = 0;
            cloud.points.push_back(p);
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;

        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud, ros_cloud);
        ros_cloud.header.frame_id = "base_link";
        ros_cloud.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "发布点云，总点数: %zu", cloud.points.size());
        point_cloud_pub_->publish(ros_cloud);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}