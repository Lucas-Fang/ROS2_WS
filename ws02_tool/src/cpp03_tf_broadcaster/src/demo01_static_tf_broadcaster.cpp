// 1.包含头文件；
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// using std::placeholders::_1;

// 4.定义节点类；
class MinimalStaticFrameBroadcaster : public rclcpp::Node
{
public:
  MinimalStaticFrameBroadcaster(char * argv[]): Node("minimal_static_frame_broadcaster")
  {
    // 4-1.创建静态坐标变换发布方对象；
    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    this->make_transforms(argv);
  }

private:
  // 4-2.组织并发布消息。
  void make_transforms(char * argv[])
  {
    // 组织消息
    geometry_msgs::msg::TransformStamped t;
    //时间戳
    rclcpp::Time now = this->get_clock()->now();
    t.header.stamp = now;

    t.header.frame_id = argv[7];//父级
    t.child_frame_id = argv[8];//子级
    //设置偏移量
    t.transform.translation.x = atof(argv[1]);
    t.transform.translation.y = atof(argv[2]);
    t.transform.translation.z = atof(argv[3]);
    //设置四元数
    tf2::Quaternion q;
    q.setRPY(
      atof(argv[4]),
      atof(argv[5]),
      atof(argv[6]));//调用内置函数将欧拉角转换为四元数
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // 发布消息
    tf_publisher_->sendTransform(t);
  }
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char * argv[])
{
  // 2.判断终端传入的参数是否合法；
  auto logger = rclcpp::get_logger("logger");

  if (argc != 9) {
    RCLCPP_ERROR(
      logger, "运行程序时请按照:x y z roll pitch yaw frame_id child_frame_id 的格式传入参数");
    return 1;
  }

  // 3.初始化 ROS 客户端；
  rclcpp::init(argc, argv);
  // 5.调用 spin 函数，并传入对象指针；
  rclcpp::spin(std::make_shared<MinimalStaticFrameBroadcaster>(argv));
  // 6.释放资源。
  rclcpp::shutdown();
  return 0;
}