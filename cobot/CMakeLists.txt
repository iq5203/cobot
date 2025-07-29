cmake_minimum_required(VERSION 3.8)
project(cobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(behaviortree_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include
${behaviortree_cpp_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/cobot.cpp src/Estop.cpp src/Stop.cpp src/Slow.cpp src/FullSpeed.cpp src/CobotNode.cpp)
ament_target_dependencies(${PROJECT_NAME} behaviortree_cpp rclcpp std_msgs)
target_link_libraries(${PROJECT_NAME} ${behaviortree_cpp_LIBRARIES})
target_include_directories(cobot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(cobot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
	cobot
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        behavior_trees
        DESTINATION share/${PROJECT_NAME}/)

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
