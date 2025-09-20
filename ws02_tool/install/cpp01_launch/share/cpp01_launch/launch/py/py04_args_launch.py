from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    decl_bg_r = DeclareLaunchArgument(name="background_r",default_value="255")
    decl_bg_g = DeclareLaunchArgument(name="background_g",default_value="255")
    decl_bg_b = DeclareLaunchArgument(name="background_b",default_value="255")

    turtle = Node(
        package="turtlesim", 
        executable="turtlesim_node",
        parameters=[{
             "background_r": LaunchConfiguration("background_r"),
             "background_g": LaunchConfiguration("background_g"),
             "background_b": LaunchConfiguration("background_b")}]
        )
    return LaunchDescription([decl_bg_r,decl_bg_g,decl_bg_b,turtle])
#ros2 launch cpp01_launch py04_args_launch.py decl_g:=0