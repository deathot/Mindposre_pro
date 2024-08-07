cmake_minimum_required(VERSION 3.8)
project(ros2_dvpp2_resize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV 4.5.4 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(INC_PATH $ENV{DDK_PATH})
if (NOT DEFINED ENV{DDK_PATH})
    set(INC_PATH "/usr/local/Ascend/ascend-toolkit/latest/arm64-linux")
    message(STATUS "set default INC_PATH: ${INC_PATH}")
else ()
    message(STATUS "env INC_PATH: ${INC_PATH}")
endif()

set(LIB_PATH $ENV{NPU_HOST_LIB})
# Dynamic libraries in the stub directory can only be used for compilation
if (NOT DEFINED ENV{NPU_HOST_LIB})
    set(LIB_PATH "/usr/local/Ascend/ascend-toolkit/latest/runtime/lib64/stub/")
    message(STATUS "set default LIB_PATH: ${LIB_PATH}")
else ()
    message(STATUS "env LIB_PATH: ${LIB_PATH}")
endif()

link_directories(
    ${LIB_PATH}
)
include_directories(
    ${INC_PATH}/runtime/include/acl/
    ${INC_PATH}/runtime/include/acl/dvpp/
    ./
)


add_executable(publisher_yuvmsg_node
    src/PublisherYuvMsg.cpp
    src/DvppProcess.cpp
    src/Utils.cpp
)

ament_target_dependencies(publisher_yuvmsg_node
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
)

add_executable(ros2_dvpp2_resize_node
    src/Ros2Dvpp2Resize.cpp
    src/DvppProcess.cpp
    src/Utils.cpp
)

ament_target_dependencies(ros2_dvpp2_resize_node
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
)

target_link_libraries(ros2_dvpp2_resize_node
    acl_dvpp_mpi
    ascendcl
    stdc++
    /lib/aarch64-linux-gnu/libtiff.so.5
)

target_link_libraries(publisher_yuvmsg_node
    acl_dvpp_mpi
    ascendcl
    stdc++
    /lib/aarch64-linux-gnu/libtiff.so.5
)

target_include_directories(ros2_dvpp2_resize_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(publisher_yuvmsg_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
    ros2_dvpp2_resize_node
    publisher_yuvmsg_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

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