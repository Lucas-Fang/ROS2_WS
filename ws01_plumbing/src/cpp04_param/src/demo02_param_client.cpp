/* 创建客户端，查询或者修改参数 */
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamClient: public rclcpp::Node
{
  public:
    ParamClient():Node("param_client_node_cpp",rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
      RCLCPP_INFO(this->get_logger(),"参数服务端创建成功!");
      //参数1:当前对象依赖的节点 this
      //参数2:参数服务端节点名称
      param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"param_server_node_cpp");
      /*
        问题:服务端通信是使用话题通信进行连接,为什么这里却使用节点名称呢?
        1.参数服务端启动之后底层封装了多个服务通信的服务端
        例:
              /param_server_node_cpp/describe_parameters
              /param_server_node_cpp/get_parameter_types
              /param_server_node_cpp/get_parameters
              /param_server_node_cpp/list_parameters
              /param_server_node_cpp/set_parameters
              /param_server_node_cpp/set_parameters_atomically
        2.每个服务端话题的命名格式,/服务端节点名称/xxx
        3.参数客户端创建之后也会封装多个底层客户端,他们一一和服务端连接;
        
        */
      
    }
//连接服务端
    bool connect_server(){
        while (!param_client_->wait_for_service(1s)) {
          
          if (!rclcpp::ok()) {
            return false;//按下ctrl c
          }
          RCLCPP_INFO(this->get_logger(),"服务连接中......");
        }
        return true;
    }
//查询参数
    void get_param(){
      RCLCPP_INFO(this->get_logger(),"----------参数查询操作--------------");

      //获得某一个参数
      std::string car_name = param_client_->get_parameter<std::string>("car_name");
      double width = param_client_->get_parameter<double>("width");
      RCLCPP_INFO(this->get_logger(),"carname = %s",car_name.c_str());
      RCLCPP_INFO(this->get_logger(),"width = %.2f",width);
      //获得多个参数
      auto params = param_client_->get_parameters({"car_name","width","wheels"});
      //使用for遍历
      for (auto &&param : params) {
        RCLCPP_INFO(this->get_logger(),"%s = %s",param.get_name().c_str(),param.value_to_string().c_str());
      }
      //判断是否包含某个参数
      RCLCPP_INFO(this->get_logger(),"包含carname吗?%d",param_client_->has_parameter("car_name"));
      RCLCPP_INFO(this->get_logger(),"包含height吗?%d",param_client_->has_parameter("height"));
    }
//修改参数
    void update_param(){
      RCLCPP_INFO(this->get_logger(),"----------参数修改操作--------------");

      param_client_->set_parameters({rclcpp::Parameter("car_name","pig"),
                  rclcpp::Parameter("width",3.0),
                  rclcpp::Parameter("length",5.0)});
    }
    private:
        rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto client = std::make_shared<ParamClient>();
  bool flag = client->connect_server();
  if (!flag) {
    return 0;
  }
  client->get_param();
  client->update_param();
  client->get_param();
  rclcpp::shutdown();
  return 0;
}