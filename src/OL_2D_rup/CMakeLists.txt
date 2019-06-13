cmake_minimum_required(VERSION 2.8.3)
project(ol_2d_rup)

add_definitions(-DLINUX)

# Tell CMake to compile with C++11
set(CMAKE_CXX_STANDARD 11)

# Tell CMake to run moc when needed.
set(CMAKE_AUTOMOC ON)

# Tell CMake to run uic when needed.
set(CMAKE_AUTOUIC ON)

# Tell CMake to run rcc when needed
set(CMAKE_AUTORCC ON)

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  #OpenCV REQUIRED
  #PCL REQUIRED
  roscpp
  roslib
  param_lib
  math_lib
  bhand_lib
)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
# Add Qt5 definitions and includes to build libraries.
# Widgets add Widgets Core and Gui
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDES})

# BHand need to know that we are on Linux
add_definitions( -DLINUX )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

set(SRC_LIST
  src/source_list/utils.cpp
  include/OL_2D_rup/GUI/gripper_gui/gripper_mainwindow.cpp
  src/source_list/Gripper/Gripper.cpp
  src/source_list/Gripper/Bhand.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/OL_2D_rup/GUI/gripper_gui
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  /usr/include/eigen3
)

# *************************************************
# *************************************************

add_executable(gripper_node
  src/gripper_node.cpp
  ${SRC_LIST}
)

add_dependencies(gripper_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(gripper_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  param-lib
  math-lib
  bhand_lib
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)
