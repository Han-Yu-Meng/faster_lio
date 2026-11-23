cmake_minimum_required(VERSION 3.16)

project(faster_lio)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)

# 发行版打包的 yaml-cpp 只导出裸目标，补一个兼容别名以保留上游写法
if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

add_compile_options(
    -Wno-reorder
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-sign-compare
    -Wno-parentheses
    -Wno-unused-but-set-variable
)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

set(LIGHTNING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lightning)

fins_add_node(${PROJECT_NAME}
  src/nodes.cpp
  src/mapper_loc.cpp
  src/params_adapter.cpp
  src/pcl_instantiate.cpp
  ${LIGHTNING_DIR}/core/lio/laser_mapping.cc
  ${LIGHTNING_DIR}/core/lio/eskf.cc
  ${LIGHTNING_DIR}/core/lio/pointcloud_preprocess.cc
  ${LIGHTNING_DIR}/common/nav_state.cc
  ${LIGHTNING_DIR}/common/options.cc
  ${LIGHTNING_DIR}/utils/timer.cc
)

if(TARGET livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp)
    set(LIVOX_MSG_PREFIX livox_interfaces2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE LIVOX_INTERFACES2)
else()
    set(LIVOX_MSG_PREFIX livox_ros_driver2)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${LIGHTNING_DIR}>
    ${PCL_INCLUDE_DIRS}
)

fins_link_ros_dependencies(${PROJECT_NAME}
    geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
    pcl_conversions
    rclcpp
    tf2
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenMP::OpenMP_CXX
    ${PCL_LIBRARIES}
    pcl_surface
    glog::glog
    yaml-cpp::yaml-cpp
    livox_ros_driver2::${LIVOX_MSG_PREFIX}__rosidl_generator_cpp
    livox_ros_driver2::${LIVOX_MSG_PREFIX}__rosidl_typesupport_cpp
)

# lightning 用自定义 PointXYZIT；PCL 默认只实例化标准点类型，需此宏让模板实现进入编译单元
target_compile_definitions(${PROJECT_NAME} PRIVATE PCL_NO_PRECOMPILE)
target_compile_definitions(${PROJECT_NAME} PRIVATE NDEBUG)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE -O3)
endif()
