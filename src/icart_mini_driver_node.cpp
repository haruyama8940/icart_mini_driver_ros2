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
        : Node("icart_mini_driver")
        {}
        // setParam();
        // getParam();
        // cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        // "cmd_vel", 10, std::bind(&Icart_mini_driver::cmd_vel_cb, this, std::placeholders::_1));
        // geometry_msgs::msg::Twist::ConstPtr cmd_vel_;
        void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
        void read_param();
        void bringup_ypspur(std::vector<std::string> args);
    
    private:
        geometry_msgs::msg::Twist::SharedPtr cmd_vel_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        // void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const;
        
};
    void Icart_mini_driver::read_param()
    {  

    }
    void Icart_mini_driver::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            cmd_vel_ =  msg;
            // https://github.com/openspur/yp-spur/blob/master/doc/Manpage.control.md#velocity-control
            YP::YPSpur_vel(msg->linear.x,msg->angular.z);
        }
        
    // void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    //  {
    //         // cmd_vel_ =  msg;
    //         // https://github.com/openspur/yp-spur/blob/master/doc/Manpage.control.md#velocity-control
    //         YP::YPSpur_vel(msg->angular.x,msg->angular.z);
    //  }
    //  if (YP::YP_get_error_state() == 0)
    //     {
            
    //     }
    // void Icart_mini_driver::bringup_ypspur(std::vector<std::string> args)
    // {
    //     std::vector<std::string> args =
    //         {
    //           ypspur_bin_,
    //           "-d", port_,
    //           "--admask", ad_mask,
    //           "--msq-key", std::to_string(key_)
    //         };
    //     system("ypspur-coordinator -p  -d /dev/sensors/icart-mini");
    // }
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Icart_mini_driver>());
  rclcpp::shutdown();
  return 0;
}
