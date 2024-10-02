cmake_minimum_required(VERSION 3.22)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

# Install Python execs
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/picknplace
  scripts/follow_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
