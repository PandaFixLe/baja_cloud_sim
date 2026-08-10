#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>   // 【新增】引入 Float32 消息类型
#include <tf2/LinearMath/Quaternion.h>

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <sstream>

class ChcnavFullNode : public rclcpp::Node
{
public:
    ChcnavFullNode() : Node("chcnav_full_node")
    {
        nmea_pub_ = this->create_publisher<std_msgs::msg::String>("/chcnav/nmea_sentence", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/chcnav/devimu", 10);
        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/chcnav/devpvt", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/chcnav/odom", 10);
        // 【新增】初始化 /imu_yaw 发布者
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/imu_yaw", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ChcnavFullNode::publish_nmea, this)
        );

        if (!open_can("vcan2")) {
            RCLCPP_ERROR(this->get_logger(), "打开 vcan2 失败！");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "ChcnavFullNode 启动成功，新增 /imu_yaw 话题！");

        read_thread_ = std::thread(&ChcnavFullNode::read_loop, this);
    }

    ~ChcnavFullNode()
    {
        if (read_thread_.joinable())
            read_thread_.join();
        if (sockfd_ >= 0) close(sockfd_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmea_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_; // 【新增】
    rclcpp::TimerBase::SharedPtr timer_;

    int sockfd_ = -1;
    std::thread read_thread_;

    double pos_lat_ = 0.0, pos_lon_ = 0.0, pos_alt_ = 0.0;
    double ang_rate_x_ = 0.0, ang_rate_y_ = 0.0, ang_rate_z_ = 0.0;
    double accel_x_ = 0.0, accel_y_ = 0.0, accel_z_ = 0.0;
    double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
    uint8_t state_ins_ = 0;

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
            if (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) continue;

            uint32_t id = frame.can_id & CAN_SFF_MASK;
            switch (id) {
                case 0x321: decode_angrate(frame.data); break;
                case 0x322: decode_accel(frame.data); break;
                case 0x323: decode_status(frame.data); break;
                case 0x32A: decode_attitude(frame.data); break;
                case 0x32C: decode_angrate_veh(frame.data); break;
                case 0x32D: decode_longitude(frame.data); break;
                case 0x32E: decode_latitude(frame.data); break;
                default: break;
            }
        }
    }

    void decode_angrate(const uint8_t* data) {
        int32_t x = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t y = ((data[2] & 0x0F) << 16) | (data[3] << 8) | data[4];
        int32_t z = (data[5] << 12) | (data[6] << 4) | (data[7] >> 4);
        if (data[0] & 0x80) x |= 0xFFF00000;
        if (data[2] & 0x08) y |= 0xFFF00000;
        if (data[5] & 0x80) z |= 0xFFF00000;
        ang_rate_x_ = x * 0.01;
        ang_rate_y_ = y * 0.01;
        ang_rate_z_ = z * 0.01;
        publish_imu();
    }

    void decode_accel(const uint8_t* data) {
        int32_t x = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t y = ((data[2] & 0x0F) << 16) | (data[3] << 8) | data[4];
        int32_t z = (data[5] << 12) | (data[6] << 4) | (data[7] >> 4);
        if (data[0] & 0x80) x |= 0xFFF00000;
        if (data[2] & 0x08) y |= 0xFFF00000;
        if (data[5] & 0x80) z |= 0xFFF00000;
        accel_x_ = x * 0.0001;
        accel_y_ = y * 0.0001;
        accel_z_ = z * 0.0001;
        publish_imu();
    }

    void decode_status(const uint8_t* data) {
        state_ins_ = data[0];
        publish_pvt(); 
    }

    void decode_attitude(const uint8_t* data) {
        yaw_   = ((data[0] << 8) | data[1]) * 0.01;
        pitch_ = (int16_t)((data[2] << 8) | data[3]) * 0.01;
        roll_  = (int16_t)((data[4] << 8) | data[5]) * 0.01;
        
        publish_imu();
        publish_odom();
        publish_yaw(); // 【新增】更新 yaw 时，单独发布
    }

    void decode_angrate_veh(const uint8_t* data) {
        int32_t x = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t y = ((data[2] & 0x0F) << 16) | (data[3] << 8) | data[4];
        int32_t z = (data[5] << 12) | (data[6] << 4) | (data[7] >> 4);
        if (data[0] & 0x80) x |= 0xFFF00000;
        if (data[2] & 0x08) y |= 0xFFF00000;
        if (data[5] & 0x80) z |= 0xFFF00000;
        ang_rate_x_ = x * 0.01;
        ang_rate_y_ = y * 0.01;
        ang_rate_z_ = z * 0.01;
        publish_odom();
    }

    void decode_longitude(const uint8_t* data) {
        int64_t lon = ((int64_t)data[0] << 56) | ((int64_t)data[1] << 48) |
                      ((int64_t)data[2] << 40) | ((int64_t)data[3] << 32) |
                      ((int64_t)data[4] << 24) | ((int64_t)data[5] << 16) |
                      ((int64_t)data[6] << 8)  | data[7];
        pos_lon_ = lon * 0.00000001;
        publish_pvt();
        publish_odom();
    }

    void decode_latitude(const uint8_t* data) {
        int64_t lat = ((int64_t)data[0] << 56) | ((int64_t)data[1] << 48) |
                      ((int64_t)data[2] << 40) | ((int64_t)data[3] << 32) |
                      ((int64_t)data[4] << 24) | ((int64_t)data[5] << 16) |
                      ((int64_t)data[6] << 8)  | data[7];
        pos_lat_ = lat * 0.00000001;
        publish_pvt();
        publish_odom();
    }

    void publish_nmea() {
        std_msgs::msg::String msg;
        std::stringstream ss;
        auto t = this->now();
        int sec = static_cast<int>(t.seconds()) % 86400;
        int hh = sec / 3600;
        int mm = (sec % 3600) / 60;
        int ss_sec = sec % 60;
        
        ss << "$GPGGA," << std::setfill('0') << std::setw(2) << hh
           << std::setw(2) << mm << std::setw(2) << ss_sec << ".00,";

        if (pos_lat_ != 0.0 && pos_lon_ != 0.0) {
            ss << std::fixed << std::setprecision(6);
            ss << std::abs(pos_lat_) << "," << (pos_lat_ > 0 ? "N" : "S") << ",";
            ss << std::abs(pos_lon_) << "," << (pos_lon_ > 0 ? "E" : "W") << ",";
            ss << (int)state_ins_ << ",";
            ss << "00,0.0," << pos_alt_ << ",M,,";
        } else {
            ss << ",,,,,,,,";
        }
        ss << "*7C";
        msg.data = ss.str();
        nmea_pub_->publish(msg);
    }

    void publish_imu() {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        msg.angular_velocity.x = ang_rate_x_;
        msg.angular_velocity.y = ang_rate_y_;
        msg.angular_velocity.z = ang_rate_z_;
        msg.linear_acceleration.x = accel_x_;
        msg.linear_acceleration.y = accel_y_;
        msg.linear_acceleration.z = accel_z_;

        tf2::Quaternion qtn;
        qtn.setRPY(roll_ * M_PI / 180.0, -pitch_ * M_PI / 180.0, yaw_ * M_PI / 180.0);
        msg.orientation.x = qtn.getX();
        msg.orientation.y = qtn.getY();
        msg.orientation.z = qtn.getZ();
        msg.orientation.w = qtn.getW();
        msg.orientation_covariance[0] = -1;
        imu_pub_->publish(msg);
    }

    void publish_pvt() {
        sensor_msgs::msg::NavSatFix msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "gps_link";
        msg.latitude = pos_lat_;
        msg.longitude = pos_lon_;
        msg.altitude = pos_alt_;

        if (pos_lat_ != 0.0 && pos_lon_ != 0.0) {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; 
            if (state_ins_ == 4 || state_ins_ == 8) {
                msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            }
        } else {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        fix_pub_->publish(msg);
    }

    void publish_odom() {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;

        tf2::Quaternion qtn;
        qtn.setRPY(roll_ * M_PI / 180.0, -pitch_ * M_PI / 180.0, yaw_ * M_PI / 180.0);
        msg.pose.pose.orientation.x = qtn.getX();
        msg.pose.pose.orientation.y = qtn.getY();
        msg.pose.pose.orientation.z = qtn.getZ();
        msg.pose.pose.orientation.w = qtn.getW();
        msg.twist.twist.angular.x = ang_rate_x_;
        msg.twist.twist.angular.y = ang_rate_y_;
        msg.twist.twist.angular.z = ang_rate_z_;
        odom_pub_->publish(msg);
    }

    // 【新增】专门发布偏航角（Yaw）
    void publish_yaw() {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(yaw_); // 单位：度（°）
        yaw_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChcnavFullNode>());
    rclcpp::shutdown();
    return 0;
}