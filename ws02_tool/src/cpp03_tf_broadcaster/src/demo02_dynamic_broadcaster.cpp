#include <rclcpp/rclcpp.hpp>//
#include <tf2/LinearMath/Quaternion.h>//
#include <tf2_ros/transform_broadcaster.h>//
#include <turtlesim/msg/pose.hpp>//

// using std::placeholders::_1;

class TFDynaBroadcaster: public rclcpp::Node
{
  public:
    TFDynaBroadcaster():Node("tf_dyna_broadcaster_node_cpp")
    {
      //创建一个动态发布器
      broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      //创建一个乌龟位姿订阅方
      pose_sub = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose",10 , 
        std::bind(&TFDynaBroadcaster::do_pose,this,std::placeholders::_1));
    }

    private:
      void do_pose(const turtlesim::msg::Pose &pose){
        //组织消息
        geometry_msgs::msg::TransformStamped ts;

        ts.header.stamp = this->now();
        ts.header.frame_id = "world";

        ts.child_frame_id = "turtle1";

        ts.transform.translation.x = pose.x;
        ts.transform.translation.y = pose.y;
        ts.transform.translation.z = 0.0;

        //欧拉角转换四元数
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, pose.theta);

        ts.transform.rotation.x = qtn.x();
        ts.transform.rotation.y = qtn.y();
        ts.transform.rotation.z = qtn.z();
        ts.transform.rotation.w = qtn.w();

        broadcaster_->sendTransform(ts);

      }
      std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
      rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<TFDynaBroadcaster>());

  rclcpp::shutdown();
  return 0;
}