cmake_minimum_required(VERSION 3.16)

project(segment_projection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(gflags REQUIRED)
find_package(Eigen3 3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(GDAL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(jsoncpp REQUIRED)

add_compile_definitions(GLOG_USE_GLOG_EXPORT GLOG_USE_GFLAGS)

find_package(glog CONFIG QUIET)
if(NOT TARGET glog::glog)
  find_path(GLOG_INCLUDE_DIR glog/logging.h)
  find_library(GLOG_LIBRARY NAMES glog)
  if(NOT GLOG_INCLUDE_DIR OR NOT GLOG_LIBRARY)
    message(FATAL_ERROR "glog not found")
  endif()
  add_library(glog::glog UNKNOWN IMPORTED)
  set_target_properties(glog::glog PROPERTIES
    IMPORTED_LOCATION "${GLOG_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}"
    INTERFACE_COMPILE_DEFINITIONS "GLOG_USE_GLOG_EXPORT;GLOG_USE_GFLAGS"
  )
  if(TARGET gflags::gflags)
    set_property(TARGET glog::glog APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES gflags::gflags)
  endif()
endif()

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PCL_INCLUDE_DIRS})

add_library(segment_projection_common
  src/common/math/eigen_related.cc
  src/common/time/time_related.cc
  src/common/utils/file.cc
)
target_link_libraries(segment_projection_common
  PUBLIC
    Eigen3::Eigen
)

add_library(segment_projection_data_loader
  src/data_loader/gac_clip_root_loader.cc
)
target_link_libraries(segment_projection_data_loader
  PUBLIC
    segment_projection_common
    jsoncpp_lib
    glog::glog
)

add_library(segment_projection_pose_parse
  src/pose_parse/inspvax_asc_parser.cc
)
target_link_libraries(segment_projection_pose_parse
  PUBLIC
    segment_projection_common
    GDAL::GDAL
    glog::glog
)

add_executable(deskew_clip_export
  src/application/deskew_clip_export.cc
)
target_link_libraries(deskew_clip_export
  PUBLIC
    segment_projection_common
    segment_projection_data_loader
    segment_projection_pose_parse
    yaml-cpp
    jsoncpp_lib
    glog::glog
    pcl_common
    pcl_io
)
