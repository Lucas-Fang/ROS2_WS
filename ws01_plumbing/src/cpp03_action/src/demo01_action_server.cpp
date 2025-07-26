#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"


using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionServer: public rclcpp::Node
{
  public:
    ProgressActionServer():Node("progress_action_server_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"action 服务端创建成功！");

      //创建动作客户端
      this->server_ = rclcpp_action::create_server<base_interfaces_demo::action::Progress>(
                 this,
                "get_sum",
                std::bind(&ProgressActionServer::handle_goal,this,_1,_2),
                std::bind(&ProgressActionServer::handle_cancle,this,_1),
                std::bind(&ProgressActionServer::handle_accepted,this,_1)      
              );
    }
// 3-2.处理请求数据 std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const typename Progress::Goal> goal)
    {
      (void)uuid;
      if (goal->num <= 1) {
        RCLCPP_INFO(this->get_logger(),"提交的数据必须大于1!");
        return rclcpp_action::GoalResponse::REJECT;
      }
      RCLCPP_INFO(this->get_logger(),"提交的数据合法！");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
// 3-3.处理取消任务请求 std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    rclcpp_action::CancelResponse handle_cancle(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
      (void) goal_handle;
      RCLCPP_INFO(this->get_logger(),"任务被取消！");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

// 3-4.生成连续反馈 // std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;

void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
{
  //生成连续反馈返回给客户端
  // void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
  // goal_handle->publish_feedback()
  //首先获取目标值然后遍历遍历之后累加，且每次循环都输出一次进度，作为连续反馈发布
  int num = goal_handle->get_goal()->num;
  int sum = 0;
  auto feedback = std::make_shared<Progress::Feedback>();

  auto result = std::make_shared<Progress::Result>();

  //设置休眠
  rclcpp::Rate rate(1.0);
  for (int i = 1; i<=num; i++) {
    sum += i ;
    double progress = i/(double)num;
    feedback->progress = progress;

    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(),"连续反馈中，进度：%.2f",progress);

    //判断是否接受到了终止进程
    if (goal_handle->is_canceling()) {
      //如果接收到了终止，那么退出程序
      result->sum  = sum;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(),"任务被取消！");
      return;

    }

    rate.sleep();
  }


  //生成最终相应结果 判断程序还在运行
  if (rclcpp::ok()) {


    result->sum = sum;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(),"最终结果：%d",sum);
  }

}
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
    {
      //新建子线程处理耗时操作
      std::thread(std::bind(&ProgressActionServer::execute,this,goal_handle)).detach();
    }

    private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<ProgressActionServer>());

  rclcpp::shutdown();
  return 0;
}
