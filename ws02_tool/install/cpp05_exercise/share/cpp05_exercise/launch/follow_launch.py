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
    # 优化 频繁使用的名称抽取为变量
    t2 = DeclareLaunchArgument(name="turtle_name",default_value="t2")
    # 生成乌龟节点
    turtle = Node(package="turtlesim",executable="turtlesim_node")
    # 生成一只新的🐢
    spawn = Node(package="cpp05_exercise" ,executable="exer01_spawn",
                 parameters=[{"turtle_name":LaunchConfiguration("turtle_name")}])
    # 分别广播两只乌龟相对于世界的坐标
    broadcaster1 = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="bro1")
    broadcaster2 = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="bro2",
                        parameters=[{"turtle":LaunchConfiguration("turtle_name")}])
    # 创建监听节点
    listener = Node(package="cpp05_exercise",executable="exer03_tf_listener",
                    parameters=[{"father_frame":LaunchConfiguration("turtle_name"),
                                 "child_frame":"turtle1"}])
    return LaunchDescription([t2,turtle,spawn,broadcaster1,broadcaster2,listener])