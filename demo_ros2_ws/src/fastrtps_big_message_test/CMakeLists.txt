cmake_minimum_required(VERSION 3.8)
project(fastrtps_big_message_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 创建发布者
add_executable(big_publisher src/big_publisher.cpp)
ament_target_dependencies(big_publisher rclcpp std_msgs)

# 创建订阅者  
add_executable(big_subscriber src/big_subscriber.cpp)
ament_target_dependencies(big_subscriber rclcpp std_msgs)

# 安装可执行文件
install(TARGETS
  big_publisher
  big_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装脚本文件
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
