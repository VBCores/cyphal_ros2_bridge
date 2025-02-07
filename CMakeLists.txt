cmake_minimum_required(VERSION 3.8)
project(cyphal_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

get_filename_component(CYPHAL_DIR "libs/libcxxcanard" ABSOLUTE)
message(STATUS "${CMAKE_CURRENT_LIST_DIR} | Using <${CYPHAL_DIR}> as directory of libcxxcanard")
add_subdirectory(${CYPHAL_DIR} ${PROJECT_BINARY_DIR}/build/libcxxcanard)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/HMIBeeper.msg"
    "msg/HMILed.msg"
    "srv/CallHMIBeeper.srv"
    "srv/CallHMILed.srv"
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(cyphal_bridge ${SOURCES})
target_include_directories(cyphal_bridge PUBLIC
    include
    src
    libs/types
    libs/libvoltbro
    libs/nlohmann
    ${CYPHAL_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
link_libraries(
    libcxxcanard
)
target_compile_features(cyphal_bridge PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(cyphal_bridge rclcpp std_msgs sensor_msgs nav_msgs rosidl_default_generators)

install(TARGETS cyphal_bridge DESTINATION lib/${PROJECT_NAME})
install(TARGETS libcxxcanard LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

ament_package()
