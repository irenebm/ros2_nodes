cmake_minimum_required(VERSION 3.8)
project(ros2_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

include_directories(include)

#
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher ${dependencies})

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber ${dependencies})

add_executable(complex_publisher src/complex_publisher.cpp)
ament_target_dependencies(complex_publisher ${dependencies})

add_executable(complex_subscriber src/complex_subscriber.cpp)
ament_target_dependencies(complex_subscriber ${dependencies})

install(TARGETS
    simple_publisher
    simple_subscriber
    complex_publisher
    complex_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
