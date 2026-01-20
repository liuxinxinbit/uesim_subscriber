cmake_minimum_required(VERSION 3.5)
project(uesim_subscriber)

if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD) 
endif()


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# add opencv support
find_package(OpenCV REQUIRED)
# add msg image
find_package(sensor_msgs REQUIRED)
# add cv_bridge 
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_executable(subscriber_uesim_vc vcf_subscriber.cpp)
ament_target_dependencies(subscriber_uesim_vc rclcpp std_msgs OpenCV sensor_msgs cv_bridge image_transport pcl_conversions)

install(TARGETS
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(TARGETS
  subscriber_uesim_vc
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
