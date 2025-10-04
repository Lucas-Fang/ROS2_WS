cmake_minimum_required(VERSION 3.8)
project(cpp01_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)

# 假设你写的库代码放在 src/my_serial_lib.cpp，头文件在 include/cpp01_serial/my_serial_lib.hpp
add_library(my_serial_lib
  src/crc16.cpp
)
# 库需要包含头文件目录
target_include_directories(my_serial_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 库的依赖
ament_target_dependencies(my_serial_lib
  rclcpp
)
  
# ---------------- 添加可执行文件 ----------------
add_executable(demo01_uart_send src/demo01_uart_send.cpp)
# 节点需要包含头文件目录
target_include_directories(demo01_uart_send PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(demo01_uart_send PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# 节点依赖  
ament_target_dependencies(
  demo01_uart_send
  "rclcpp"
  "serial_driver"
)
# 让节点链接你写的库
target_link_libraries(demo01_uart_send my_serial_lib)

# ---------------- 安装 ----------------
install(TARGETS 
  my_serial_lib
  demo01_uart_send
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
