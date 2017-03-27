cmake_minimum_required(VERSION 2.8.11)

project(XamlaMoveit)

find_package(Torch REQUIRED)
find_package(catkin REQUIRED)

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmax-errors=3 -Wall" )


install(DIRECTORY "./" DESTINATION "${Torch_INSTALL_LUA_PATH_SUBDIR}/xamlamoveit" FILES_MATCHING PATTERN "*.lua")

catkin_package()

install(DIRECTORY msgs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
