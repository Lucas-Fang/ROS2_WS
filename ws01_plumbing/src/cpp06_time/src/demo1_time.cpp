#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
class MyNode: public rclcpp::Node
{
  public:
    MyNode():Node("timenode_node_cpp")
    {
      // demo_rate();
      // demo_time();
      // demo_duration();
      time_opt();
    }

    private:
    int a;
    void time_opt(){
      rclcpp::Time t1(10,0);//dan wei "ns"
      rclcpp::Time t2(20,0);//dan wei "ns"    

      rclcpp::Duration du1(8,0);
      rclcpp::Duration du2(17,0);

      RCLCPP_INFO(this->get_logger(),"t1 >= t2 ? %d",t1 >= t2);
      RCLCPP_INFO(this->get_logger(),"t1 < t2 ? %d",t1 < t2);

      rclcpp::Duration du3 = t2 - t1;
      rclcpp::Time t3 = t1 + du1;
      rclcpp::Time t4 = t1 - du1;
      RCLCPP_INFO(this->get_logger(),"du3 = %.2f",du3.seconds());//10c
      RCLCPP_INFO(this->get_logger(),"t3 = %.2f",t3.seconds());//18c
      RCLCPP_INFO(this->get_logger(),"t3 = %.2f",t4.seconds());//2c

      rclcpp::Duration du4 = du1 * 3;
      RCLCPP_INFO(this->get_logger(),"du4 = %.2f",du4.seconds());//10c


    }

    void demo_duration(){
      //创建 duration 对象
      rclcpp::Duration du1(1s);
      rclcpp::Duration du2(2,500000000L);

      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %.ld",du1.seconds(),du1.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %.ld",du2.seconds(),du2.nanoseconds());
    }

    void demo_time(){
      //创建 time 对象 1s == 10亿 ns
      rclcpp::Time t1(500000000L);//dan wei "ns"
      rclcpp::Time t2(2,500000000L);//dan wei "ns"
      // rclcpp::Time right_now = this->get_clock()->now();
      rclcpp::Time right_now = this->now();

      //调用 time 对象函数
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %.ld",t1.seconds(),t1.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %.ld",t2.seconds(),t2.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %.ld",right_now.seconds(),right_now.nanoseconds());
    }

    void demo_rate(){
      //创建Rate对象 两种方式(直接写循环时间或者是频率)
      rclcpp::Rate rate1(500ms);
      rclcpp::Rate rate2(1);  //hz/s 一秒运行几次

      while (rclcpp::ok()) {
        a++;
        RCLCPP_INFO(this->get_logger(),"%d",a);
        // rate1.sleep();
        rate2.sleep();
      }

    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<MyNode>());

  rclcpp::shutdown();
  return 0;
}