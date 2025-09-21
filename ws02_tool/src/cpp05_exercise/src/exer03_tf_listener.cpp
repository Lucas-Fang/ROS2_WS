#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

class Exer03TFLinstener: public rclcpp::Node
{
  public:
    Exer03TFLinstener():Node("exer03_tf_linstener_node_cpp")
    {
      //声明参数服务
      this->declare_parameter("father_frame","turtle2");
      this->declare_parameter("child_frame","turtle1");
      father_frame = this->get_parameter("father_frame").as_string();
      child_frame = this->get_parameter("child_frame").as_string();
      //创建缓存
      buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
      //创建速度发布方
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/"+father_frame+"/cmd_vel", 10);
      //创建一个定时器
      timer_ = this->create_wall_timer(1s, std::bind(&Exer03TFLinstener::on_timer,this));
    }
    private:
    void on_timer(){
      try {
        //实时坐标变换
        auto ts = buffer_ ->lookupTransform(father_frame,child_frame,tf2::TimePointZero);

        geometry_msgs::msg::Twist twist;
        //计算线速度
        twist.linear.x = 0.6 * sqrt(
          pow(ts.transform.translation.x,2) +pow(ts.transform.translation.y,2));
        //计算角速度
        twist.angular.z = 1.0 * (atan2(
          ts.transform.translation.y,
          ts.transform.translation.x));

          // RCLCPP_INFO(this->get_logger(),"目标角度:%.2f",(-atan2(
          // ts.transform.translation.y,
          // ts.transform.translation.x)));

        cmd_pub_->publish(twist);
      } catch (const tf2::LookupException &e) {
        RCLCPP_INFO(this->get_logger(),"异常提示:%.s",e.what());
      }
    }
    std::string father_frame;
    std::string child_frame;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Exer03TFLinstener>());

  rclcpp::shutdown();
  return 0;
}