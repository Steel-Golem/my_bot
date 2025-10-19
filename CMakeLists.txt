cmake_minimum_required(VERSION 3.8)
project(my_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake (ROS 2 build system)
find_package(ament_cmake REQUIRED)

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

# Install URDF, meshes, and launch files
# (adjust folders as needed — add/remove lines based on your package structure)
install(
  DIRECTORY
    launch
    urdf
    meshes
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# If you have Python scripts in a `scripts` folder:
# install(PROGRAMS
#   scripts/some_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Export package to the ament build system
ament_package()
