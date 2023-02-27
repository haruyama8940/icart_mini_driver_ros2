#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace YP
{
#include <ypspur,h>
}

class Icart_mini_driver : public rclcpp::Node
{
    public:
        Icart_mini_driver()
        : Node("icart_moni_driver")
        {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Icart_mini_driver::cmd_vel_cb, this, _1));
        geometry_msgs::msg::Twist::ConstPtr cmd_vel_;
        }
    private:
        void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const
        {
            cmd_vel_ msg
        }
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_
}