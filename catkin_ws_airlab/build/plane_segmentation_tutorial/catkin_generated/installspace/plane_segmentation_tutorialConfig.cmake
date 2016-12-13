# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(plane_segmentation_tutorial_CONFIG_INCLUDED)
  return()
endif()
set(plane_segmentation_tutorial_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(plane_segmentation_tutorial_SOURCE_PREFIX /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/src/plane_segmentation_tutorial)
  set(plane_segmentation_tutorial_DEVEL_PREFIX /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/devel)
  set(plane_segmentation_tutorial_INSTALL_PREFIX "")
  set(plane_segmentation_tutorial_PREFIX ${plane_segmentation_tutorial_DEVEL_PREFIX})
else()
  set(plane_segmentation_tutorial_SOURCE_PREFIX "")
  set(plane_segmentation_tutorial_DEVEL_PREFIX "")
  set(plane_segmentation_tutorial_INSTALL_PREFIX /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/install)
  set(plane_segmentation_tutorial_PREFIX ${plane_segmentation_tutorial_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'plane_segmentation_tutorial' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(plane_segmentation_tutorial_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/usr/include/eigen3;/usr/local/include/pcl-1.8;/usr/include;/usr/include/ni;/usr/include/vtk-6.2;/usr/include/freetype2;/usr/include/x86_64-linux-gnu/freetype2;/usr/include/x86_64-linux-gnu;/usr/include/hdf5/openmpi;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent/include;/usr/lib/openmpi/include;/usr/lib/openmpi/include/openmpi;/usr/include/jsoncpp;/usr/include/python2.7;/usr/include/libxml2;/usr/include/tcl " STREQUAL " ")
  set(plane_segmentation_tutorial_INCLUDE_DIRS "")
  set(_include_dirs "/usr/include/eigen3;/usr/local/include/pcl-1.8;/usr/include;/usr/include/ni;/usr/include/vtk-6.2;/usr/include/freetype2;/usr/include/x86_64-linux-gnu/freetype2;/usr/include/x86_64-linux-gnu;/usr/include/hdf5/openmpi;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent;/usr/lib/openmpi/include/openmpi/opal/mca/event/libevent2021/libevent/include;/usr/lib/openmpi/include;/usr/lib/openmpi/include/openmpi;/usr/include/jsoncpp;/usr/include/python2.7;/usr/include/libxml2;/usr/include/tcl")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${plane_segmentation_tutorial_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'plane_segmentation_tutorial' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'aeroscout <aeroscout@todo.todo>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'plane_segmentation_tutorial' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/install/${idir}'.  Ask the maintainer 'aeroscout <aeroscout@todo.todo>' to fix it.")
    endif()
    _list_append_unique(plane_segmentation_tutorial_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "optimized;/usr/local/lib/libpcl_common.so;debug;/usr/local/lib/libpcl_common.so;optimized;/usr/local/lib/libpcl_octree.so;debug;/usr/local/lib/libpcl_octree.so;optimized;/usr/local/lib/libpcl_io.so;debug;/usr/local/lib/libpcl_io.so;optimized;/usr/local/lib/libpcl_kdtree.so;debug;/usr/local/lib/libpcl_kdtree.so;optimized;/usr/local/lib/libpcl_search.so;debug;/usr/local/lib/libpcl_search.so;optimized;/usr/local/lib/libpcl_sample_consensus.so;debug;/usr/local/lib/libpcl_sample_consensus.so;optimized;/usr/local/lib/libpcl_filters.so;debug;/usr/local/lib/libpcl_filters.so;optimized;/usr/local/lib/libpcl_features.so;debug;/usr/local/lib/libpcl_features.so;optimized;/usr/local/lib/libpcl_ml.so;debug;/usr/local/lib/libpcl_ml.so;optimized;/usr/local/lib/libpcl_segmentation.so;debug;/usr/local/lib/libpcl_segmentation.so;optimized;/usr/local/lib/libpcl_surface.so;debug;/usr/local/lib/libpcl_surface.so;optimized;/usr/local/lib/libpcl_registration.so;debug;/usr/local/lib/libpcl_registration.so;optimized;/usr/local/lib/libpcl_keypoints.so;debug;/usr/local/lib/libpcl_keypoints.so;optimized;/usr/local/lib/libpcl_tracking.so;debug;/usr/local/lib/libpcl_tracking.so;optimized;/usr/local/lib/libpcl_recognition.so;debug;/usr/local/lib/libpcl_recognition.so;optimized;/usr/local/lib/libpcl_stereo.so;debug;/usr/local/lib/libpcl_stereo.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/usr/lib/x86_64-linux-gnu/libpthread.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;optimized;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;debug;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND plane_segmentation_tutorial_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND plane_segmentation_tutorial_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND plane_segmentation_tutorial_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/catkin_ws_airlab/install/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(plane_segmentation_tutorial_LIBRARY_DIRS ${lib_path})
      list(APPEND plane_segmentation_tutorial_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'plane_segmentation_tutorial'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND plane_segmentation_tutorial_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(plane_segmentation_tutorial_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${plane_segmentation_tutorial_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;std_msgs;sensor_msgs;visualization_msgs;pcl_ros;pcl_conversions")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 plane_segmentation_tutorial_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${plane_segmentation_tutorial_dep}_FOUND)
      find_package(${plane_segmentation_tutorial_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${plane_segmentation_tutorial_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(plane_segmentation_tutorial_INCLUDE_DIRS ${${plane_segmentation_tutorial_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(plane_segmentation_tutorial_LIBRARIES ${plane_segmentation_tutorial_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${plane_segmentation_tutorial_dep}_LIBRARIES})
  _list_append_deduplicate(plane_segmentation_tutorial_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(plane_segmentation_tutorial_LIBRARIES ${plane_segmentation_tutorial_LIBRARIES})

  _list_append_unique(plane_segmentation_tutorial_LIBRARY_DIRS ${${plane_segmentation_tutorial_dep}_LIBRARY_DIRS})
  list(APPEND plane_segmentation_tutorial_EXPORTED_TARGETS ${${plane_segmentation_tutorial_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${plane_segmentation_tutorial_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
