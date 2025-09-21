#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TFPointListener: public rclcpp::Node
{
  public:
    TFPointListener():Node("tfpoint_listener_node_cpp")
    {
        //创建坐标变换监听器(只能监听坐标系变化无法监听坐标点的变化,所以下方创建监听坐标点的订阅方)        
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        //为buuffer创建一个定时器
        timer_ = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()
        );
        buffer_ ->setCreateTimerInterface(timer_);
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        //创建坐标点消息的订阅方
        point_sub.subscribe(this,"point");
        //创建过滤器

//     F & f,    订阅对象
//     BufferT & buffer,  坐标监听的缓存
//     const std::string & target_frame,  目标:base_link
//     uint32_t queue_size,     队列长度 10
//     const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
//     const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
//     std::chrono::duration<TimeRepT, TimeT> buffer_timeout = 超时时间
//     std::chrono::duration<TimeRepT, TimeT>::max())
        filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
            point_sub,
            *buffer_,
            "base_link",
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface(),
            1s
        );
        //解析数据
        filter_ ->registerCallback(&TFPointListener::transform_point ,this);
    }
    private:
    void transform_point(const geometry_msgs::msg::PointStamped & ps){
        //实现坐标点的变换
        //必须包含头文件 "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 不然编译失败
        auto out = buffer_->transform(ps,"base_link");
        RCLCPP_INFO(this->get_logger(),"父级坐标系:%s ,坐标:(%.2f,%.2f,%.2f)",
        out.header.frame_id.c_str(),
        out.point.x,
        out.point.y,
        out.point.z
    );

    }
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;
        std::shared_ptr<tf2_ros::CreateTimerROS> timer_;
        message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub;
        std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> filter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<TFPointListener>());

  rclcpp::shutdown();
  return 0;
}