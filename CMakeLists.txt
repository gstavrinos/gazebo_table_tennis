cmake_minimum_required(VERSION 3.8)
project(gazebo_table_tennis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(tiago_robot REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#ament_python_install_package(${PROJECT_NAME})

ament_package()

install(DIRECTORY scripts/
      DESTINATION lib/${PROJECT_NAME}
      FILES_MATCHING
      PATTERN "*.py"
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})
