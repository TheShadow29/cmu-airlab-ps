# Install script for directory: /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/src/plane_segmentation_tutorial

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/build/plane_segmentation_tutorial/catkin_generated/installspace/plane_segmentation_tutorial.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plane_segmentation_tutorial/cmake" TYPE FILE FILES
    "/home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/build/plane_segmentation_tutorial/catkin_generated/installspace/plane_segmentation_tutorialConfig.cmake"
    "/home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/build/plane_segmentation_tutorial/catkin_generated/installspace/plane_segmentation_tutorialConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/plane_segmentation_tutorial" TYPE FILE FILES "/home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/src/plane_segmentation_tutorial/package.xml")
endif()

