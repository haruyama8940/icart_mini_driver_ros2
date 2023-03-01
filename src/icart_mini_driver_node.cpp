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
        Icart_mini_driver(): Node("icart_mini_driver")
        {
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",1);
            // odom_pub_timer_ = this->create_wall_timer(500ms,std::bind(&Icart_mini_driver::odometry,this));
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&Icart_mini_driver::cmd_vel_cb, this, std::placeholders::_1));
        }
        // setParam();
        // getParam();
      
        // geometry_msgs::msg::Twist::ConstPtr cmd_vel_;
        void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
        void read_param();
        void bringup_ypspur(std::vector<std::string> args);
        // void odometry();
        // void jointstate();
        bool loop();
        
    
    private:
        geometry_msgs::msg::Twist::SharedPtr cmd_vel_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        // void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const;
        
};
    // this function is read ypspur_param file from yaml
    void Icart_mini_driver::read_param()
    {  

    }
    // this function is subscribe cmd_vel and send command to ypspur
    void Icart_mini_driver::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            cmd_vel_ =  msg;
            RCLCPP_INFO(this->get_logger(),"sub cmd_vel");
            // https://github.com/openspur/yp-spur/blob/master/doc/Manpage.control.md#velocity-control
            // YP::YPSpur_vel(msg->linear.x,msg->angular.z);
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
    //this function is set ypspur_param and bringup ypspur_coordinator
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
    // main loop
    // void Icart_mini_driver::odometry()
    // {
    //     nav_msgs::msg::Odometry odom;
    //     odom.header.frame_id;
    //     odom.child_frame_id;
    //     odom.pose.pose.position.x = 0;
    //     odom.pose.pose.position.y = 0;
    //     odom.pose.pose.position.z = 0;
    //     // odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, 0));
    //     odom.twist.twist.linear.x = 0;
    //     odom.twist.twist.linear.y = 0;
    //     odom.twist.twist.angular.z = 0;
    //     odom_pub_->publish(odom);
    // }
    bool Icart_mini_driver::loop()
    {
         RCLCPP_INFO(this->get_logger(),"sub cmd_vel");
        // int state = YP::YP_get_error_state();

        // if (state == 0){
        // }
        // else {
        //     RCLCPP_WARN(this->get_logger(),"Disconnected T-frog driver");
        //     // bringup_ypspur();
        // }

     return true;
    }
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Icart_mini_driver>());
  rclcpp::shutdown();
  return 0;
}
