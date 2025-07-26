#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"
#include <base_interfaces_demo/msg/detail/student__struct.hpp>

using base_interfaces_demo::msg::Student;

class LinternerStu: public rclcpp::Node
{
  public:
    LinternerStu():Node("lintenerstu_node_cpp")
    {
    subscription_ = this->create_subscription<Student>("chatter_stu", 10, std::bind(&LinternerStu::do_cb,this,std::placeholders::_1));
    }
  private:
    void do_cb(const base_interfaces_demo::msg::Student &stu)
    {
        RCLCPP_INFO(this->get_logger(),"收到的信息为：%s,%d,%.2f",stu.name.c_str(),stu.age,stu.height);
    }
    rclcpp::Subscription<Student>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<LinternerStu>());

  rclcpp::shutdown();
  return 0;
}


