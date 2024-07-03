cmake_minimum_required(VERSION 3.8)
project(MujocoRosUtils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set Mujoco Enviroments
set(MUJOCO_ROOT_DIR "/home/ws/mujoco-2.3.5/")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/lib")
set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin")
set(MUJOCO_REQUIRED_VERSION 235)

# Include headers
set(MUJOCO_MAIN_HEADER "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
if(NOT EXISTS "${MUJOCO_MAIN_HEADER}")
  message(FATAL_ERROR "MuJoCo main header (${MUJOCO_MAIN_HEADER}) not found, maybe MUJOCO_ROOT_DIR is badly set")
endif()
file(READ "${MUJOCO_MAIN_HEADER}" MUJOCO_MAIN_HEADER_CONTENT)
if(MUJOCO_MAIN_HEADER_CONTENT MATCHES ".*#define mjVERSION_HEADER ([0-9]+).*")
  set(MUJOCO_VERSION ${CMAKE_MATCH_1})
else()
  message(FATAL_ERROR "Could not extract MuJoCo version from the header, please contact this package's developers")
endif()
if(${MUJOCO_VERSION} LESS ${MUJOCO_REQUIRED_VERSION})
  message(FATAL_ERROR "This package requires at least MuJoCo version ${MUJOCO_REQUIRED_VERSION} but you have ${MUJOCO_VERSION}")
endif()

file(GLOB LIB_MUJOCO ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9].so ${MUJOCO_LIB_DIR}/libmujoco.so.*)
if(EXISTS ${LIB_MUJOCO})
  message(STATUS "MuJoCo lib found: ${LIB_MUJOCO}")
else()
  message(FATAL_ERROR "MuJoCo lib not found.")
endif()

# Include directories
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
)

# GLFW
find_package(glfw3 3.3 REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

add_library(
  ${PROJECT_NAME} SHARED
  ${source})

target_link_libraries(${PROJECT_NAME} ${LIB_MUJOCO} ${catkin_LIBRARIES} glfw)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIB_MUJOCO})
# 
target_compile_options(${PROJECT_NAME} PUBLIC ${DEEPBREAK_CXX_FLAGS})  
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME}
  "ament_cmake"
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "cv_bridge"
)


# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

# Export package
ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


# Ament package
ament_package()