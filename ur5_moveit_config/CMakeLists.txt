cmake_minimum_required(VERSION 3.22)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

# Install Python execs
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/world_bottle/picknplace_bottle_low
  scripts/world_bottle/picknplace_bottle_same
  scripts/world_bottle/picknplace_cube_low
  scripts/world_bottle/picknplace_cube_same
  scripts/world_bottle_high/picknplace_cube_high
  scripts/world_pot_faucet/picknplace_pot_low
  scripts/world_pot_faucet/picknplace_faucet_same
  scripts/world_pot_faucet/picknplace_faucet_low
  scripts/world_pot_faucet_high/picknplace_faucet_high
  scripts/code_template/code_template.py
  scripts/code_template/template_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
