#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"



using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;

class Talkerstu: public rclcpp::Node
{
  public:
  Talkerstu():Node("talkerstu_node_cpp")
    {
    //创建发布方
      publisher_ = this->create_publisher<Student>("chatter_stu", 10);
        
      timer_ = this->create_wall_timer(500ms, std::bind(&Talkerstu::on_timer,this));
    
    }

    private:
    void on_timer()
    {
        //组织发布信息
        auto stu = Student();
        stu.name = "erGouzi";
        stu.age  = 20 ;
        stu.height = 22.2;
        
        publisher_ ->publish(stu);

        RCLCPP_INFO(this->get_logger(),"发布的消息为：%s,%d,%.2f",stu.name.c_str(),stu.age,stu.height);

    }
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Talkerstu>());

  rclcpp::shutdown();
  return 0;
}