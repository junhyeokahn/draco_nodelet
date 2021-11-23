cmake_minimum_required(VERSION 3.0.2)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_BUILD_TYPE)

project(draco_nodelet)

set(B_FIXED_CONFIGURATION False)

set(CMAKE_CXX_STANDARD 17)
include_directories("/usr/local/include")
link_directories("/usr/local/lib")
list(APPEND CMAKE_MODULE_PATH "/home/apptronik/Repository/PnC/cmake")

find_package(catkin REQUIRED COMPONENTS cortex_utils message_generation message_runtime nodelet pluginlib roscpp roslib rt_utils tf2_eigen nodelet)
find_package(Boost QUIET REQUIRED COMPONENTS system thread filesystem)
if(B_FIXED_CONFIGURATION)
    find_package(FixedDracoPnC REQUIRED)
else()
    find_package(DracoPnC REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cortex_utils message_generation message_runtime nodelet pluginlib roscpp roslib rt_utils tf2_eigen
  )

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if(B_FIXED_CONFIGURATION)
    include_directories(${FIXED_DRACO_PNC_INCLUDE_DIRS})
else()
    include_directories(${DRACO_PNC_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -fPIC -Wno-unused-result -Wno-overflow -Dunix -rdynamic")

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/draco_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(B_FIXED_CONFIGURATION)
    target_link_libraries(${PROJECT_NAME} ${FIXED_DRACO_PNC_LIBRARIES})
    message(${FIXED_DRACO_PNC_INCLUDE_DIRS})
    message(${FIXED_DRACO_PNC_LIBRARIES})
else()
    target_link_libraries(${PROJECT_NAME} ${DRACO_PNC_LIBRARIES})
    message(${DRACO_PNC_INCLUDE_DIRS})
    message(${DRACO_PNC_LIBRARIES})
endif()

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/include/draco_nodelet/nodelet_configuration.hpp)
