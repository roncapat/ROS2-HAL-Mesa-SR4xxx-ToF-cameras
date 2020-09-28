cmake_minimum_required(VERSION 3.5)
project(hal_tof_mesa_sr4xxx)

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

# NEEDED TO WORK IN CLION (CMAKE-BASED IDEs)
include_directories(/opt/ros/$ENV{ROS_CHOOSE_DISTRO}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../build/${PROJECT_NAME}/rosidl_generator_cpp)
link_directories(/opt/ros/$ENV{ROS_CHOOSE_DISTRO}/lib)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../build/${PROJECT_NAME})
set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_CHOOSE_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_CHOOSE_DISTRO}/lib/libfastrtps.so)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(perception_pcl REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetAmplitudeThreshold.srv"
  "srv/GetPointCloud.srv"
  "srv/SetAutoExposure.srv"
  "srv/SetNonAmbiguityMode.srv"
  "srv/GetDeviceInfo.srv"
  "srv/GetSerial.srv"
  "srv/SetDualIntegration.srv"
  "srv/SetTimeout.srv"
  "srv/GetIntegrationTime.srv"
  "srv/GetTimeout.srv"
  "srv/SetIntegrationTime.srv"
  "srv/GetModulationFrequency.srv"
  "srv/SetAmplitudeThreshold.srv"
  "srv/SetModulationFrequency.srv"
   DEPENDENCIES sensor_msgs
 )

find_package(Boost COMPONENTS system)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )


add_executable(HALToFMesaSR4xxx src/HALToFMesaSR4xxx.cpp)
add_dependencies(HALToFMesaSR4xxx hal_tof_mesa_sr4xxx)
rosidl_target_interfaces(HALToFMesaSR4xxx ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(HALToFMesaSR4xxx rclcpp rcutils perception_pcl pcl_conversions sensor_msgs)
target_link_libraries(HALToFMesaSR4xxx mesasr)
target_link_libraries(HALToFMesaSR4xxx ${PCL_LIBRARIES})
target_link_libraries(HALToFMesaSR4xxx Boost::system)

target_include_directories(HALToFMesaSR4xxx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_COMMON_INCLUDE_DIRS} 
  $<INSTALL_INTERFACE:include>)

install(TARGETS HALToFMesaSR4xxx
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
