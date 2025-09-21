#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <turtlesim/srv/detail/spawn__struct.hpp>

using namespace std::chrono_literals;
class Exer01Spawn: public rclcpp::Node
{
  public:
    Exer01Spawn():Node("exer01_spawn_node_cpp")
    {
      this->declare_parameter("x",3.0);
      this->declare_parameter("y",3.0);
      this->declare_parameter("theta",0.0);
      this->declare_parameter("turtle_name","turtle2");
    
      x = this->get_parameter("x").as_double();
      y = this->get_parameter("y").as_double();
      theta = this->get_parameter("theta").as_double();
      turtle_name = this->get_parameter("turtle_name").as_string();

      //创建服务客户端
      spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
    }
    bool connect_server(){
      while (!spawn_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强制推出!");
          return false;
        }
        RCLCPP_INFO(this->get_logger(),"服务连接中...");
        
      }
      return true;
    }
    //组织并发送数据
    rclcpp::Client<turtlesim::srv::Spawn>::FutureAndRequestId request(){

      auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
      req->x = x;
      req->y = y;
      req->theta = theta;
      req->name = turtle_name;
      return spawn_client_->async_send_request(req);
    }

    private:
    double_t x,y,theta;
    std::string turtle_name;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto client = std::make_shared<Exer01Spawn>();
  bool flag = client->connect_server();
  if (!flag) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接失败");
  }

  //发送请求
  auto response = client->request();
  //处理响应
  if (rclcpp::spin_until_future_complete(client,response) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(client->get_logger(),"相应成功!");
    //生成乌龟时如果重名了会返回一个空的字符串是
    std::string name = response.get()->name;
    if (name.empty())
    {
        RCLCPP_INFO(client->get_logger(),"乌龟重名导致生成失败！");
    } else {
        RCLCPP_INFO(client->get_logger(),"乌龟%s生成成功！", name.c_str());
    }
  }else {
    RCLCPP_INFO(client->get_logger(),"响应失败!");
  }

  rclcpp::shutdown();
  return 0;
}
