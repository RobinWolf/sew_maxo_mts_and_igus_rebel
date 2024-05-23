cmake_minimum_required(VERSION 3.8)
project(sew_agv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(navigation2 REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_bt_navigator REQUIRED)    #and many more nav2_* packages, but is some are installed correctly, they are all!
find_package(slam_toolbox REQUIRED)
find_package(rviz2 REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(twist_mux REQUIRED)





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

# add custom directories to install them while buildung
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
