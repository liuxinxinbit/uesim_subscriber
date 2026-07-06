cmake_minimum_required(VERSION 3.5)
project(uesim_subscriber)

if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD) 
endif()


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robots_dog_msgs REQUIRED)

# C++ 可执行文件
add_executable(subscriber_uesim_vc vcf_subscriber.cpp)
target_include_directories(subscriber_uesim_vc PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(subscriber_uesim_vc ${OpenCV_LIBS})
ament_target_dependencies(subscriber_uesim_vc rclcpp std_msgs sensor_msgs nav_msgs robots_dog_msgs)

# Python 包安装
ament_python_install_package(${PROJECT_NAME})

# Python 脚本安装
install(PROGRAMS
    ${PROJECT_NAME}/sensor_subscriber.py
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  subscriber_uesim_vc
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
