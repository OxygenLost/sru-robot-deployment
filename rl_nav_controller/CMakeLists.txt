cmake_minimum_required(VERSION 3.8)
project(rl_nav_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# ONNX Runtime setup (architecture-specific)
# ============================================================================
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" RLC_SYSTEM_PROCESSOR)
set(ONNXRUNTIME_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
if(RLC_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
  set(ONNXRUNTIME_ROOT "${ONNXRUNTIME_BASE_DIR}/aarch64")
else()
  set(ONNXRUNTIME_ROOT "${ONNXRUNTIME_BASE_DIR}/x86_64")
endif()

if(NOT EXISTS "${ONNXRUNTIME_ROOT}")
  message(FATAL_ERROR "ONNX Runtime directory not found: ${ONNXRUNTIME_ROOT}")
endif()

set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_BASE_DIR}/include")
if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIRS}")
  message(FATAL_ERROR "ONNX Runtime include directory not found: ${ONNXRUNTIME_INCLUDE_DIRS}")
endif()

find_library(ONNXRUNTIME_LIBRARIES
  NAMES onnxruntime
  PATHS
    "${ONNXRUNTIME_ROOT}/lib"
    "${ONNXRUNTIME_ROOT}/lib64"
  NO_DEFAULT_PATH
)

if(NOT ONNXRUNTIME_LIBRARIES)
  message(FATAL_ERROR "libonnxruntime.so not found under ${ONNXRUNTIME_ROOT}/lib or lib64")
endif()

get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARIES}" DIRECTORY)
file(GLOB ONNXRUNTIME_RUNTIME_LIBS
  "${ONNXRUNTIME_LIBRARY_DIR}/libonnxruntime*.so*"
  "${ONNXRUNTIME_LIBRARY_DIR}/libonnxruntime_providers_shared.so*"
)

# Create ONNX Runtime imported target
add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIRS}"
)

# ============================================================================
# Find dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ============================================================================
# C++ executable: depth_preprocess_node (existing)
# ============================================================================
add_executable(depth_preprocess_node
  src/depth_preprocess.cpp
)

ament_target_dependencies(depth_preprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(depth_preprocess_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS depth_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# C++ executable: rl_nav_controller_node (new - ONNX inference)
# ============================================================================
add_executable(rl_nav_controller_node
  src/navigation_policy_node.cpp
  src/learning_model.cpp
  src/visualization_manager.cpp
  src/waypoint_manager.cpp
)

target_include_directories(rl_nav_controller_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ONNXRUNTIME_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(rl_nav_controller_node PRIVATE
  onnxruntime::onnxruntime
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(rl_nav_controller_node PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  ament_index_cpp
)

set_target_properties(rl_nav_controller_node PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH "\$ORIGIN"
)

target_compile_features(rl_nav_controller_node PUBLIC cxx_std_17)

install(TARGETS rl_nav_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${ONNXRUNTIME_RUNTIME_LIBS}
  DESTINATION lib/${PROJECT_NAME})

# ============================================================================
# Python module (kept for reference / multi_point_nav)
# ============================================================================
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  rl_nav_controller/rl_nav_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rl_nav_controller_py
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install deployment policies (ONNX models)
install(DIRECTORY deployment_policies/
  DESTINATION share/${PROJECT_NAME}/deployment_policies
)

ament_package()
