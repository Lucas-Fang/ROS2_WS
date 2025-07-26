#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>


using namespace std::chrono_literals;


class Talker: public rclcpp::Node{
  public:
        Talker():Node("talker_node"),count(0){
          RCLCPP_INFO(this->get_logger(),"发布节点创建！"); 

          publisher = this->create_publisher<std_msgs::msg::String>("chatter",10);

          //创建定时器
          timer_ = this->create_wall_timer(1s, std::bind(&Talker::on_timer,this));
        }

  private:
        void on_timer(){
          auto message =std_msgs::msg::String();
          message.data = "hello_world" + std::to_string(count++);
          publisher->publish(message);

          RCLCPP_INFO(this->get_logger(),":发布放发布的消息：%s",message.data.c_str());
        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count;//设置一个计数器

};
int main(int argc,char ** argv)
{
  rclcpp::init(argc, argv);
 
  rclcpp::spin(std::make_shared<Talker>());

  rclcpp::shutdown();
  return 0;
}
