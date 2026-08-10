#include "rclcpp/rclcpp.hpp"

#include "msg_interfaces/msg/hc_sentence.hpp"
#include "msg_interfaces/msg/hcinspvatzcb.hpp"
#include "msg_interfaces/msg/hcrawimub.hpp"
#include "hc_cgi_protocol.h"

static rclcpp::Publisher<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr gs_devpvt_pub;
static rclcpp::Publisher<msg_interfaces::msg::Hcrawimub>::SharedPtr gs_devimu_pub;
static std::shared_ptr<rclcpp::Node> g_node;

static void hc_sentence_callback(const msg_interfaces::msg::HcSentence::ConstPtr msg);

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("HcCgiProtocolProcessNode");
    std::shared_ptr<rclcpp::Node> private_nh = g_node;

    auto serial_suber = g_node->create_subscription<msg_interfaces::msg::HcSentence>("/chcnav/hc_sentence", 1000, hc_sentence_callback);

    gs_devpvt_pub = g_node->create_publisher<msg_interfaces::msg::Hcinspvatzcb>("/chcnav/devpvt", 1000);
    gs_devimu_pub = g_node->create_publisher<msg_interfaces::msg::Hcrawimub>("/chcnav/devimu", 1000);

    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}

static void msg_deal__hcinspvatzcb(const msg_interfaces::msg::HcSentence::ConstPtr &msg);
static void msg_deal__hcrawimuib(const msg_interfaces::msg::HcSentence::ConstPtr &msg);

/**
 * @brief 处理华测协议的回调函数
 * */
static void hc_sentence_callback(const msg_interfaces::msg::HcSentence::ConstPtr msg)
{
    // 【拦截清除1：彻底注释掉 CRC 校验，保证任何数据都能进来】
    // if (hc__cgi_check_crc32(...) ...)

    switch (msg->msg_id)
    {
        case INSPVATZCB: // 0x1201
            msg_deal__hcinspvatzcb(msg);
            break;
        case RAWIMUIB:   // 0x0102
            msg_deal__hcrawimuib(msg);
            break;
        default:
            break;
    }
}

static void msg_deal__hcinspvatzcb(const msg_interfaces::msg::HcSentence::ConstPtr &msg)
{
    msg_interfaces::msg::Hcinspvatzcb devpvt;
    devpvt.header = msg->header;

    // 【拦截清除2：不管长度是 296 还是 297，只要大于 200 就认为是有效数据】
    if (msg->data.size() > 200) 
    {
        // 【拦截清除3：强制使用当前系统时间，完全抛弃容易出错的闰秒计算】
        devpvt.header.stamp = g_node->now();

        devpvt.latitude = *((double *)(&msg->data[32]));
        devpvt.longitude = *((double *)(&msg->data[40]));
        devpvt.altitude = *((float *)(&msg->data[48]));
        devpvt.roll = *((float *)(&msg->data[72]));
        devpvt.pitch = *((float *)(&msg->data[68]));
        devpvt.yaw = *((float *)(&msg->data[76]));
        devpvt.speed = *((float *)(&msg->data[140]));

        // 强制发布
        gs_devpvt_pub->publish(devpvt);
    }
}

static void msg_deal__hcrawimuib(const msg_interfaces::msg::HcSentence::ConstPtr &msg)
{
    msg_interfaces::msg::Hcrawimub devimu;
    devimu.header = msg->header;

    // 【拦截清除4：不管长度是 34 还是 68，只要长度大于 30 就强制解析】
    if (msg->data.size() > 30) 
    {
        devimu.header.stamp = g_node->now();

        // 角速度 (单位转换由解析时实现)
        devimu.angular_velocity.x = ((float)*((short *)(&msg->data[14]))) / 80.0;
        devimu.angular_velocity.y = ((float)*((short *)(&msg->data[16]))) / 80.0;
        devimu.angular_velocity.z = ((float)*((short *)(&msg->data[18]))) / 80.0;

        devimu.linear_acceleration.x = ((float)*((short *)(&msg->data[20]))) / 5000.0;
        devimu.linear_acceleration.y = ((float)*((short *)(&msg->data[22]))) / 5000.0;
        devimu.linear_acceleration.z = ((float)*((short *)(&msg->data[24]))) / 5000.0;

        // 强制发布
        gs_devimu_pub->publish(devimu);
    }
}