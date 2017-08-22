cmake_minimum_required(VERSION 2.8.3)
project(hexapod_ik)

# Tell CMake where to find "FindXXX.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(catkin REQUIRED
  COMPONENTS
    trac_ik_lib
    tf
)

find_package(orocos_kdl REQUIRED)

find_package(HexapodController REQUIRED) # COMPONENTS hexapod_controller_simple)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
    tf
  DEPENDS
    orocos_kdl
  INCLUDE_DIRS
    include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${HexapodController_INCLUDE_DIRS}
)

add_executable(hexapod_ik src/trac_ik_test.cpp)
target_link_libraries(hexapod_ik
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS hexapod_ik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(multipod_test src/multipod_ik_test.cpp)
target_link_libraries(multipod_test
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
