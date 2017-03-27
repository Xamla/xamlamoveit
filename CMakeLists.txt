cmake_minimum_required(VERSION 2.8.11)

project(XamlaMoveit)

find_package(Torch REQUIRED)
#find_package(Boost 1.47.0 REQUIRED COMPONENTS system thread chrono)
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmax-errors=3 -Wall" )

set(src
  "${SOURCE_DIR}/msgs/action/moveJ.action"
  "${SOURCE_DIR}/msgs/action/moveL.action"
  "${SOURCE_DIR}/msgs/action/moveP.action"
)

install(DIRECTORY "./" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/xamlamoveit" FILES_MATCHING PATTERN "*.lua")
