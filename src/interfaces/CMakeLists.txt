cmake_minimum_required(VERSION 3.8)
project(interfaces)

# Enable strict warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---

# Core ament build system
find_package(ament_cmake REQUIRED)

# Required to generate C++/Python code from .action/.msg/.srv files
find_package(rosidl_default_generators REQUIRED)

# --- Interface generation ---

# Generate code from the Navigate action definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigate.action"
)

# Finalize the ament package
ament_package()
