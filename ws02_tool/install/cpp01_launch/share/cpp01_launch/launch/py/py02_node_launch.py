from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
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
from ament_index_python.packages import get_package_share_directory
import os

"""

        package:功能包；
        executable:可执行文件；
        namespace:命名空间；
        name:节点名称；
        exe_name:流程标签；
        respawn:设置为True时,关闭节点后,可以自动重启。
        parameters:导入参数。
        remappings:话题重映射。
        arguments:调用指令时的参数列表。
        ros_arguments:相当于 arguments 前缀 --ros-args。

"""

def generate_launch_description():
    # turtle1 = Node(
    #     package="turtlesim",
    #     executable="turtlesim_node",
    #     exec_name="mylabel",
    #     ros_arguments=["--remp","_ns:=/t2"]
    #     #相当于ros2 run turtlesim turtlesim_node --ros-args -rempa __ns:=/t1
    #     )
    turtle2 = Node(package="turtlesim", 
                executable="turtlesim_node", 
                name="t2",
                #是否重启节点(在错误关闭之后),
                respawn=True,   #是否自动重启              
                # 参数设置方式1
                # parameters=[{"background_r": 0,"background_g": 0,"background_b": 0}],
                # 参数设置方式2: 从 yaml 文件加载参数，yaml 文件所属目录需要在配置文件中安装。
                parameters=[os.path.join(get_package_share_directory("cpp01_launch"),"config","t2.yaml")],

                )
    
    return LaunchDescription([turtle2])