#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "tf"

namespace YP
{
#include <ypspur.h>
}

class Icart_mini_driver : public rclcpp::Node
{
    public:
        Icart_mini_driver()
        : Node("icart_moni_driver")
        {
        // setParam();
        // getParam();
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Icart_mini_driver::cmd_vel_cb, this, std::placeholders::_1));
        // geometry_msgs::msg::Twist::ConstPtr cmd_vel_;
        }
    private:
        geometry_msgs::msg::Twist::SharedPtr cmd_vel_;
        // void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const;
        void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            cmd_vel_ =  msg;
            // https://github.com/openspur/yp-spur/blob/master/doc/Manpage.control.md#velocity-control
            YP::YPSpur_vel(msg->angular.x,msg->angular.z);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};
    // void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    //  {
    //         // cmd_vel_ =  msg;
    //         // https://github.com/openspur/yp-spur/blob/master/doc/Manpage.control.md#velocity-control
    //         YP::YPSpur_vel(msg->angular.x,msg->angular.z);
    //  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Icart_mini_driver>());
  rclcpp::shutdown();
  return 0;
}
