#include "rclcpp/rclcpp.hpp"
// #include <rclcpp/node_options.hpp>
// #include <rclcpp/parameter.hpp>

class ParamServer: public rclcpp::Node
{
  public:
// 允许删除参数必须加 NodeOptions
    ParamServer():Node("param_server_node_cpp",rclcpp::NodeOptions().allow_undeclared_parameters(true))
    {
      RCLCPP_INFO(this->get_logger(),"参数服务端创建成功!");
    }
    // 普通的节点即可作为服务端
    //增
    void declare_param()
    {
        RCLCPP_INFO(this->get_logger(),"-----------add---------");

        this->declare_parameter("car_name","tiger");
        this->declare_parameter("width",2.55);
        this->declare_parameter("wheels",4);
    }
    //查
   void get_param()
    {
        RCLCPP_INFO(this->get_logger(),"-----------cha---------");

        // 获取指定
        rclcpp::Parameter car_type = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(),"car_name:%s",car_type.as_string().c_str());
        RCLCPP_INFO(this->get_logger(),"width:%.2f",this->get_parameter("width").as_double());
        RCLCPP_INFO(this->get_logger(),"wheels:%ld",this->get_parameter("wheels").as_int());
       
        // 获取所有
        auto params = this->get_parameters({"car_name","width","wheels"});
        for (auto &param : params)
        {
            RCLCPP_INFO(this->get_logger(),"name = %s, value = %s", param.get_name().c_str(), param.value_to_string().c_str());

        }

        // 判断包含
        RCLCPP_INFO(this->get_logger(),"是否包含car_name? %d",this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"是否包含car_typesxxxx? %d",this->has_parameter("car_typexxxx"));
    }
    //改
   void update_param()
    {
        RCLCPP_INFO(this->get_logger(),"-----------gai---------");

        this->set_parameter(rclcpp::Parameter("width",1.75));
        RCLCPP_INFO(this->get_logger(),"width = %.2f",this->get_parameter("width").as_double());
        this->set_parameter(rclcpp::Parameter("height",2.0));
        RCLCPP_INFO(this->get_logger(),"height = %.2f",this->get_parameter("height").as_double());

    }
    //删
    void del_param()
    {
        RCLCPP_INFO(this->get_logger(),"-----------shan---------");

        //不能删除声明的参数🧬（有declare_parameter（）声明的），可以删除set_parammeter新增的
        this->undeclare_parameter("height");
        RCLCPP_INFO(this->get_logger(),"删除后还包含height吗？%d",this->has_parameter("height"));


    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<ParamServer>();

  node->declare_param();
  node->get_param();
  node->update_param();
  node->del_param();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}