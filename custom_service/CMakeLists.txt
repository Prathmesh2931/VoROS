cmake_minimum_required(VERSION 3.8)
project(custom_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(ament_cmake_python REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Text.srv"
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

install(
  DIRECTORY
    srv
    src
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)


install(PROGRAMS
  src/movebot_text.py
  src/angle_tune.py
  src/speech_to_text.py
  src/speech_to_command.py
  src/yaw.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
