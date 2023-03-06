#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Cmdvelsub : public rclcpp::Node
{
public:
    Cmdvelsub() : Node("cmd_vel_subscriber")
    {
        // cmd_vel トピックをサブスクライブする
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Cmdvelsub::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"sub cmd_vel");
        // double linear_vel_x = msg->linear.x;
        // double angular_vel_z = msg->angular.z;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Cmdvelsub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
