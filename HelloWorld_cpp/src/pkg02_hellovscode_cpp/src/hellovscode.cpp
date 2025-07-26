#include "rclcpp/rclcpp.hpp"
#include <rclcpp/utilities.hpp>

//方式一  直接实例化node（不推荐）

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc,argv);

//   auto node = rclcpp::Node::make_shared("hellovscode_node");

//   RCLCPP_INFO(node->get_logger(),"hello_vscode!");

//   return 0;
// }

//方式二 （推荐）
//使用继承的方式创建类
class Mynode: public rclcpp::Node{
  public:
         Mynode():Node("hellovscode_node"){
            RCLCPP_INFO(this->get_logger(),"hello_vscode!(继承方式)" );
        }
};

int main(int argc, char const *argv[])
{
  //初始化
  rclcpp::init(argc,argv);
  //实例化自定义类
  auto node = std::make_shared<Mynode>();
  //释放
  rclcpp::shutdown();
  return 0;
}
