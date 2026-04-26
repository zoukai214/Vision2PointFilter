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

set(segment_projection_projection_sources
  src/projection/camera_calibration.cc
  src/projection/image_index.cc
  src/projection/point_cloud_projector.cc
)
set(segment_projection_projection_all_sources_present TRUE)
foreach(segment_projection_projection_source IN LISTS segment_projection_projection_sources)
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${segment_projection_projection_source}")
    set(segment_projection_projection_all_sources_present FALSE)
  endif()
endforeach()

if(segment_projection_projection_all_sources_present)
  find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)

  add_library(segment_projection_projection
    ${segment_projection_projection_sources}
  )
  target_link_libraries(segment_projection_projection
    PUBLIC
      segment_projection_common
      segment_projection_data_loader
      jsoncpp_lib
      glog::glog
      ${OpenCV_LIBS}
  )
  target_include_directories(segment_projection_projection
    PUBLIC
      ${PROJECT_SOURCE_DIR}/src
      ${OpenCV_INCLUDE_DIRS}
  )
else()
  # Keep Task 1 configurable: provide the projection target name without
  # forcing future source files or OpenCV into the current configure/build.
  add_library(segment_projection_projection INTERFACE)
  target_link_libraries(segment_projection_projection
    INTERFACE
      segment_projection_common
      segment_projection_data_loader
      jsoncpp_lib
      glog::glog
  )
  target_include_directories(segment_projection_projection
    INTERFACE
      ${PROJECT_SOURCE_DIR}/src
  )
endif()

add_executable(deskew_clip_export
  src/application/deskew_clip_export.cc
)
target_link_libraries(deskew_clip_export
  PUBLIC
    segment_projection_common
    segment_projection_data_loader
    segment_projection_pose_parse
    segment_projection_projection
    yaml-cpp
    jsoncpp_lib
    glog::glog
    pcl_common
    pcl_io
)

enable_testing()

if(EXISTS "${PROJECT_SOURCE_DIR}/tests/projection/camera_calibration_test.cc")
  add_executable(camera_calibration_test
    tests/projection/camera_calibration_test.cc
  )
  target_link_libraries(camera_calibration_test
    PRIVATE
      segment_projection_projection
      jsoncpp_lib
      glog::glog
  )
  add_test(NAME camera_calibration_test COMMAND camera_calibration_test)
endif()

if(EXISTS "${PROJECT_SOURCE_DIR}/tests/projection/image_index_test.cc")
  add_executable(image_index_test
    tests/projection/image_index_test.cc
  )
  target_link_libraries(image_index_test
    PRIVATE
      segment_projection_projection
      glog::glog
  )
  add_test(NAME image_index_test COMMAND image_index_test)
endif()

if(EXISTS "${PROJECT_SOURCE_DIR}/tests/projection/point_cloud_projector_test.cc")
  add_executable(point_cloud_projector_test
    tests/projection/point_cloud_projector_test.cc
  )
  target_link_libraries(point_cloud_projector_test
    PRIVATE
      segment_projection_projection
      pcl_common
      glog::glog
  )
  add_test(NAME point_cloud_projector_test COMMAND point_cloud_projector_test)
endif()
