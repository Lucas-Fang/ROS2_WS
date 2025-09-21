from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,LaunchConfiguration
from launch.actions import DeclareLaunchArgument

#示例： ros2 launch cpp06_urdf display.launch.py model:=`ros2 pkg prefix --share cpp06_urdf`/urdf/urdf/demo01_helloworld.urdf
def generate_launch_description():

    cpp06_urdf_dir = get_package_share_directory("cpp06_urdf")
    default_model_path = os.path.join(cpp06_urdf_dir,"urdf/urdf","demo01_helloworld.urdf")
    default_rviz_path = os.path.join(cpp06_urdf_dir,"rviz","urdf.rviz")
    model = DeclareLaunchArgument(name="model", default_value=default_model_path)

    # 加载机器人模型
    # 1.启动 robot_state_publisher 节点并以参数方式加载 urdf 文件
    robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]))
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    # 2.启动 joint_state_publisher 节点发布非固定关节状态
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    # rviz2 节点
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz_path]
    )
    return LaunchDescription([
        model,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])

##########################-------------------------------------#####################################
# def generate_launch_description():
#     # 启动 robot_state_publisher 节点,该节点要以参数服务的方式加载urdf文件 "xacro "这后面有一个空格,不打会报错!!!
#     p_value = ParameterValue(Command(["xacro ","/home/xiaofang/ROS2_WS/ws02_tool/install/cpp06_urdf/share/cpp06_urdf/urdf/urdf/demo01_helloworld.urdf"
#             ]))#get_package_share_directory("cpp06_urdf")+"/urdf/urdf/demo01_helloworld.urdf"
#     robot_state_pub = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{"robot_description":p_value}]
#     )
#     # 2.启动rviz2
#     rviz2 = Node(
#         package="rviz2",
#         executable="rviz2"
#         # arguments=["-d", default_rviz_path]
#     )
#     return LaunchDescription([robot_state_pub,rviz2])