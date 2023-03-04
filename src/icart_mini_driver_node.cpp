#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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
        // void bringup_ypspur(std::vector<std::string> args);
        void bringup_ypspur();
        void odometry();
        // void jointstate();
        bool loop();
        
    
    private:
        geometry_msgs::msg::Twist::SharedPtr cmd_vel_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        nav_msgs::msg::Odometry odom;
        //devel odom
        // float dt = 1.0 / 0.5;
        // double tf_time_offset_ =0.0;
        tf2::Vector3 z_axis_;
        
        // tf2_ros::TransformBroadcaster tf_broadcaster_();
        // geometry_msgs::msg::TransformStamped odom_trans;

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
        YP::YPSpur_vel(msg->linear.x,msg->angular.z);
    }
    //this function is set ypspur_param and bringup ypspur_coordinator
    // void Icart_mini_driver::bringup_ypspur(std::vector<std::string> args)
    void Icart_mini_driver::bringup_ypspur()
    {
    //     std::vector<std::string> args =
    //         {
    //           ypspur_bin_,
    //           "-d", port_,
    //           "--admask", ad_mask,
    //           "--msq-key", std::to_string(key_)
    //         };
    //     system("ypspur-coordinator -p  -d /dev/sensors/icart-mini");
        RCLCPP_INFO(this->get_logger(),"Bringup ypspur!!");
        // YP::YPSpur_set_vel(0.0);
    
    }
    // main loop
    //this function is compute odometry and pub odometry topic , odom tf
    // void Icart_mini_driver::odometry()
    // {
        
    //     double x,y,yaw,v,w;
    //     z_axis_.setX(0);
    //     z_axis_.setY(0);
    //     z_axis_.setZ(1);
    //     rclcpp::Time t = this->now();
    //     const rclcpp::Time current_stamp(t);
    //     v = cmd_vel_->linear.x;
    //     w = cmd_vel_->angular.z;
    //     yaw = tf2::getYaw(odom.pose.pose.orientation) + dt * w;
    //     x = odom.pose.pose.position.x + dt * v * cosf(yaw);
    //     y = odom.pose.pose.position.y + dt * v * sinf(yaw);
    //     // odom.header.stamp = current_stamp;
    //     odom.pose.pose.position.x = x;
    //     odom.pose.pose.position.y = y;
    //     odom.pose.pose.position.z = 0;
    //     odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, yaw));
    //     odom.twist.twist.linear.x = v;
    //     odom.twist.twist.linear.y = 0;
    //     odom.twist.twist.angular.z = w;
    //     odom_pub_->publish(odom);

    //     //odom_tf
    //     odom_trans.header.stamp = current_stamp + rclcpp::Duration::from_seconds(tf_time_offset_);
    //     odom_trans.transform.translation.x = x;
    //     odom_trans.transform.translation.y = y;
    //     odom_trans.transform.translation.z = 0;
    //     odom_trans.transform.rotation = odom.pose.pose.orientation;
    //     // tf_broadcaster_.sendTransform(odom_trans);
    // }
    
    bool Icart_mini_driver::loop()
    {
        z_axis_.setX(0);
        z_axis_.setY(0);
        z_axis_.setZ(1);
        // double liner_x = 0.0;
        // double angular = 0.0;
        // cmd_vel_->linear.x = 0.0;
        // cmd_vel_->angular.z  = 0.0;
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, 0));
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;
        bool test_flag = true;
        // double dt = 1.0/0.5;
        bringup_ypspur();
        while (test_flag)
        {
          RCLCPP_INFO(this->get_logger(),"Start ypspur loop!!");
          odometry();
          YP::YPSpur_vel(cmd_vel_->linear.x,cmd_vel_->angular.z);
        }
        
        // int state = YP::YP_get_error_state();
        // if (state == 0){
        //   // odometry();
        //   // YP::YPSpur_vel(cmd_vel_->linear.x,cmd_vel_->angular.z);
        // }
        // else {
        //     RCLCPP_WARN(this->get_logger(),"Disconnected T-frog driver");
        //     // bringup_ypspur();
        //     return false;
        // }

     return true;
    }
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
//   Icart_mini_driver icart;
  auto icart = std::make_shared<Icart_mini_driver>();
// //   rclcpp::shutdown();

//   while (rclcpp::ok())
//   {
//     icart.loop();

//   }
  
  int ret=0;

  try
  {
    // Icart_mini_driver icart;
    if (!icart->loop())
      ret = 1;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    ret = 1;
  }
  
  // return 0;

  return ret;
}
