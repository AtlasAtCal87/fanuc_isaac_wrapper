cmake_minimum_required(VERSION 3.5)
project(fanuc_isaac_wrapper)

find_package(ament_cmake REQUIRED)

# THIS IS THE CRITICAL PART YOU ARE LIKELY MISSING:
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
