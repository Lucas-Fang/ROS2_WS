#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class PointBroadcaster: public rclcpp::Node
{
  public:
    PointBroadcaster():Node("point_broadcaster_node_cpp"),x()
    {
      point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point",10);
      //创建定时器
      timer_ = this->create_wall_timer(0.1s,std::bind(&PointBroadcaster::on_timer, this));
    }

    private:
    //定时器回调函数
    void on_timer(){
        geometry_msgs::msg::PointStamped ps;

        point_pub_->publish(ps);
        ps.header.stamp = this->now();
        ps.header.frame_id = "laser";
        x+=0.05;
        ps.point.x = x;
        ps.point.y = 0.0;
        ps.point.z = -0.1;

        point_pub_->publish(ps);
    }
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double_t x;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<PointBroadcaster>());

  rclcpp::shutdown();
  return 0;
}