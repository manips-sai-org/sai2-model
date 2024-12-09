cmake_minimum_required(VERSION 3.10)

project(SAI-MODEL)
set(PROJECT_VERSION 0.1.0) # major version is zero for now since api will
                           # certainly change in future

set(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -fPIC")
if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/include")
endif()

# set default build to release with debug info. mimic'ed from RBDL project.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  # Set the default build, and also the possible values of build type for
  # cmake-gui
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TESTS "Build tests" OFF)

# include Eigen
find_package(Eigen3 REQUIRED)

# include RBDL
set(RBDL_LOCAL_DIR ${PROJECT_SOURCE_DIR}/rbdl)
set(RBDL_INCLUDE_DIR ${RBDL_LOCAL_DIR}/include ${RBDL_LOCAL_DIR}/build/include)

# include sai-URDF
find_package(SAI-URDF REQUIRED)

# include Parser
set(PARSER_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/parser)
set(PARSER_SOURCE ${PROJECT_SOURCE_DIR}/src/parser/UrdfToSaiModel.cpp
                  ${PROJECT_SOURCE_DIR}/src/parser/SaiModelParserUtils.cpp)

# include Model
set(SAI-MODEL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
set(MODEL_SOURCE ${PROJECT_SOURCE_DIR}/src/SaiModel.cpp
                 ${PROJECT_SOURCE_DIR}/src/RBDLExtensions.cpp)

# Add the include directory to the include paths
include_directories(
  ${SAI-MODEL_INCLUDE_DIRS}
  ${SAI-URDF_INCLUDE_DIRS}
  ${PARSER_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${RBDL_INCLUDE_DIR}
)

# Create the library
add_library(sai-model STATIC ${MODEL_SOURCE} ${PARSER_SOURCE})

find_library(
  RBDL_LIBRARIES rbdl
  PATHS ${RBDL_LOCAL_DIR}/build
  NO_DEFAULT_PATH)
set(SAI-MODEL_LIBRARIES sai-model ${RBDL_LIBRARIES})

# set the urdf model folder
set(URDF_MODELS_FOLDER ${PROJECT_SOURCE_DIR}/urdf_models)
file(
  WRITE ${CMAKE_CURRENT_BINARY_DIR}/../src/parser/SaiModel_UrdfFolder.h
  "#ifndef SAI_MODEL_URDF_FOLDER_H_
#define SAI_MODEL_URDF_FOLDER_H_

#define SAI_MODEL_URDF_FOLDER \"${URDF_MODELS_FOLDER}\"

#endif // SAI_MODEL_URDF_FOLDER_H_")

#
# export package
#

# export package for use from the build tree
export(TARGETS sai-model FILE ${PROJECT_BINARY_DIR}/SAI-MODELTargets.cmake)
export(PACKAGE SAI-MODEL)

# configure package
set(CONF_INCLUDE_DIRS ${SAI-MODEL_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR})
set(CONF_LIBRARIES ${SAI-MODEL_LIBRARIES})

# create package configuration and version files
configure_file(SAI-MODELConfig.cmake.in
               "${PROJECT_BINARY_DIR}/SAI-MODELConfig.cmake" @ONLY)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI-MODELConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

if(BUILD_EXAMPLES)
  # add examples
  add_subdirectory(${PROJECT_SOURCE_DIR}/examples)
endif()

if(BUILD_TESTS)
  # add tests
  add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
endif()
