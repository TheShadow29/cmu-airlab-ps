cmake_minimum_required(VERSION 2.8.3)
project(stereo_assignment)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED COMPONENTS common io)

catkin_package(
   DEPENDS OpenCV Boost Eigen PCL
#  INCLUDE_DIRS include
#  LIBRARIES stereo_assignment
   CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
set (LIBELAS_SRC_DIR src/elas)
include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
					${LIBELAS_SRC_DIR}
					)
SET(CMAKE_CXX_FLAGS "-msse3")				  
FILE(GLOB LIBELAS_SRC_FILES "src/elas/*.cpp")
				  
## Declare a cpp executable
add_executable(stereo_assignment_main src/stereo_code.cpp ${LIBELAS_SRC_FILES})
# add_executable(stereo_match src/stereo_match.cpp)
target_link_libraries( stereo_assignment_main 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES})
# target_link_libraries( stereo_match
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#     ${Boost_LIBRARIES})

