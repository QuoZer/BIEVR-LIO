cmake_minimum_required(VERSION 3.16)
project(bievr_lio VERSION 0.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Build configuration
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Build a shared library by default (matches the previous catkin behaviour).
option(BUILD_SHARED_LIBS "Build bievr_lio as a shared library" ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Dependencies
#
# Ceres / glog / Eigen are currently provided by the *_catkin source packages
# in the workspace devel/install space, so they are found via CMAKE_PREFIX_PATH
# when the workspace is sourced. TBB is found system-wide.
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)
find_package(glog QUIET)  # usually re-exported transitively by Ceres

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/bievr_map.cpp
  src/common.cpp
  src/pipeline.cpp
  src/imu_integrator.cpp
  src/inertial_factor.cpp
  src/ls_optimizer.cpp
  src/preprocess.cpp
  src/prior_factor.cpp
  src/synchronizer.cpp
  src/timing.cpp
  src/undistort.cpp
  src/utils.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# C++20 with concepts. GCC < 10 spells C++20 as c++2a and needs -fconcepts.
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:GNU>:-fconcepts>
  $<$<CONFIG:Release>:-O3>
)

# Route all logging through log++ (include/bievr_lio/log++.h) in LPP mode. PUBLIC
# so consumers that include our headers compile log++ in the same mode.
target_compile_definitions(${PROJECT_NAME} PUBLIC MODE_LPP)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  Ceres::ceres
  TBB::tbb
)
if(glog_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC glog::glog)
endif()

# ---------------------------------------------------------------------------
# Install & export
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Standalone tools
#
# bumpmap_from_pcd builds a .bumpmap directly from an accumulated point cloud
# (see tools/bumpmap_from_pcd.cpp / docs/plans/SPEC-bumpmap-from-pcd.md). It
# needs PCL's readers for PCD/PLY input, which the core library otherwise has
# no use for, so PCL and yaml-cpp (config_loader.h) are found and linked only
# for this executable -- the bievr_lio library keeps its current dependency
# set (Eigen / Ceres / TBB / glog).
# ---------------------------------------------------------------------------
option(BIEVR_BUILD_TOOLS "Build standalone conversion tools (needs PCL)" ON)
if(BIEVR_BUILD_TOOLS)
  # Ubuntu's vtk9 package (pulled in transitively by PCL's io component) refers
  # to an MPI::MPI_C imported target in its own exported VTK-targets.cmake, but
  # only defines it if MPI was already found first -- otherwise CMake errors
  # out inside VTK's config file before PCL's own (optional) VTK handling ever
  # runs. Finding MPI up front sidesteps that packaging quirk.
  # find_package(MPI ... COMPONENTS C) needs the C language enabled in this
  # CMake project, which `project(... LANGUAGES CXX)` above does not do.
  enable_language(C)
  find_package(MPI COMPONENTS C QUIET)
  find_package(PCL REQUIRED COMPONENTS common io)
  find_package(yaml-cpp REQUIRED)

  add_executable(bumpmap_from_pcd tools/bumpmap_from_pcd.cpp)
  set_target_properties(bumpmap_from_pcd PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  target_compile_options(bumpmap_from_pcd PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-fconcepts>
    $<$<CONFIG:Release>:-O3>
    ${PCL_DEFINITIONS}
  )
  target_include_directories(bumpmap_from_pcd PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_directories(bumpmap_from_pcd PRIVATE ${PCL_LIBRARY_DIRS})
  target_link_libraries(bumpmap_from_pcd PRIVATE
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
    yaml-cpp::yaml-cpp
  )

  install(TARGETS bumpmap_from_pcd
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

set(BIEVR_LIO_HAS_GLOG ${glog_FOUND})
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
