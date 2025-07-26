#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"
#include <base_interfaces_demo/srv/detail/add_ints__struct.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>


using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

class AddIntsClient: public rclcpp::Node
{
  public:
    AddIntsClient():Node("addintsclient_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"客户端创建!");
      //创建客户端
      client_ = this->create_client<AddInts>("add_ints");
    }
    //连接服务函数
    bool connect_server()
    {
      //在指定时间内连接服务器，如果连接上了，返回true,否则返false
      while (!client_->wait_for_service(1s)) {//循环知道连接到服务器
      //ctrl+c被按下
        if (!rclcpp::ok()) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强行终止客户端!");
          return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务器连接中……");
      }
       return true;
    }
    //编写发送请求函数
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1,int num2)
    {
      auto request = std::make_shared<AddInts::Request>();
      request->num1 = num1;
      request->num2 = num2;
      return  client_->async_send_request(request);
    }

    private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  if (argc != 3) {
    //通过rclcpp输出日志，无需节点
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请提交两个整形数据");
    return 1;
  }

  rclcpp::init(argc,argv);

  auto client = std::make_shared<AddIntsClient>();
  bool flag = client->connect_server();

  if (!flag) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务器连接失败，程序退出！");
  }

  auto future = client->send_request(atoi(argv[1]),atoi(argv[2]));
  if (rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(client->get_logger(),"响应成功！ sum = %d",future.get()->sum);
  }
  else {
    RCLCPP_INFO(client->get_logger(),"响应失败1");
  }

//客户端无需使用和回旋函数
//   rclcpp::spin(std::make_shared<AddIntsClient>());

  rclcpp::shutdown();
  return 0;
}