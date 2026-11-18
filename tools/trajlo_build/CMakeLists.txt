cmake_minimum_required(VERSION 3.16)
project(trajlo_headless LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)
set(TRAJLO_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../third_party/traj_lo")
add_subdirectory("${TRAJLO_SOURCE}/thirdparty/ros" rosbag)
add_executable(trajlo_headless
  ../trajlo_headless.cpp
  "${TRAJLO_SOURCE}/src/io/data_loader.cpp"
  "${TRAJLO_SOURCE}/src/utils/config.cpp"
  "${TRAJLO_SOURCE}/src/core/odometry.cpp"
  "${TRAJLO_SOURCE}/src/core/map_manager.cpp")
target_compile_definitions(trajlo_headless PRIVATE SOPHUS_USE_BASIC_LOGGING)
target_include_directories(trajlo_headless PRIVATE
  "${TRAJLO_SOURCE}/include" "${SOPHUS_INCLUDE_DIR}" "${ROBIN_INCLUDE_DIR}")
target_link_libraries(trajlo_headless PRIVATE Eigen3::Eigen TBB::tbb yaml-cpp rosbag pthread)
