cmake_minimum_required(VERSION 3.8)
project(b2w_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" B2W_SYSTEM_PROCESSOR)
set(ONNXRUNTIME_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
if(B2W_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
  set(ONNXRUNTIME_ROOT "${ONNXRUNTIME_BASE_DIR}/aarch64")
else()
  set(ONNXRUNTIME_ROOT "${ONNXRUNTIME_BASE_DIR}/x86_64")
endif()

if(NOT EXISTS "${ONNXRUNTIME_ROOT}")
  message(FATAL_ERROR "ONNX Runtime directory not found: ${ONNXRUNTIME_ROOT}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Manually configure ONNX Runtime
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
if(NOT ONNXRUNTIME_RUNTIME_LIBS)
  message(FATAL_ERROR "No ONNX Runtime shared libraries found in ${ONNXRUNTIME_LIBRARY_DIR}")
endif()

# Create ONNX Runtime imported target
add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIRS}"
)

add_executable(b2w_controllers src/b2w_controllers.cpp)
set_target_properties(b2w_controllers PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH "\$ORIGIN"
)

# Add include directories
target_include_directories(b2w_controllers PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ONNXRUNTIME_INCLUDE_DIRS}
)

# Link ONNX Runtime library
target_link_libraries(b2w_controllers PRIVATE onnxruntime::onnxruntime)

# Use ament_target_dependencies to properly set up ROS 2 dependencies
ament_target_dependencies(b2w_controllers PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY src/policy/
  DESTINATION share/${PROJECT_NAME}/policy
)

target_compile_features(b2w_controllers PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS b2w_controllers
  DESTINATION lib/${PROJECT_NAME})
install(FILES ${ONNXRUNTIME_RUNTIME_LIBS}
  DESTINATION lib/${PROJECT_NAME})

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
