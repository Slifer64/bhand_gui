cmake_minimum_required(VERSION 2.8.3)

project(bhand_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DLINUX)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

file(GLOB BUILD_TARGER_FILES "src/*.cpp" "API/src/*.cpp" "API/src/linux/*.cpp")

include_directories(
  include
  API
  API/ctb-0.14
  API/ctb-0.14/linux
  #API/src
  #API/src/linux
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
)

add_library(${PROJECT_NAME} SHARED
  ${BUILD_TARGER_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pcan
  PocoFoundation
  pthread
  ${catkin_LIBRARIES}
)
