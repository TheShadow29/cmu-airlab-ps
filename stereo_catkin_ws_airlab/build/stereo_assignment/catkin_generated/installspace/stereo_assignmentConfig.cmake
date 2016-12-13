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


if(stereo_assignment_CONFIG_INCLUDED)
  return()
endif()
set(stereo_assignment_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(stereo_assignment_SOURCE_PREFIX /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/stereo_catkin_ws_airlab/src/stereo_assignment)
  set(stereo_assignment_DEVEL_PREFIX /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/stereo_catkin_ws_airlab/devel)
  set(stereo_assignment_INSTALL_PREFIX "")
  set(stereo_assignment_PREFIX ${stereo_assignment_DEVEL_PREFIX})
else()
  set(stereo_assignment_SOURCE_PREFIX "")
  set(stereo_assignment_DEVEL_PREFIX "")
  set(stereo_assignment_INSTALL_PREFIX /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/stereo_catkin_ws_airlab/install)
  set(stereo_assignment_PREFIX ${stereo_assignment_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'stereo_assignment' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(stereo_assignment_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/opt/ros/kinetic/include/opencv;/opt/ros/kinetic/include;/usr/include;/usr/include/eigen3;/usr/local/include/pcl-1.8;/usr/include/ni " STREQUAL " ")
  set(stereo_assignment_INCLUDE_DIRS "")
  set(_include_dirs "/opt/ros/kinetic/include/opencv;/opt/ros/kinetic/include;/usr/include;/usr/include/eigen3;/usr/local/include/pcl-1.8;/usr/include/ni")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${stereo_assignment_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'stereo_assignment' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'dmaturan <dimatura@cmu.edu>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'stereo_assignment' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/stereo_catkin_ws_airlab/install/${idir}'.  Ask the maintainer 'dmaturan <dimatura@cmu.edu>' to fix it.")
    endif()
    _list_append_unique(stereo_assignment_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/usr/local/lib/libopencv_xphoto.so.3.1.0;/usr/local/lib/libopencv_xobjdetect.so.3.1.0;/usr/local/lib/libopencv_ximgproc.so.3.1.0;/usr/local/lib/libopencv_xfeatures2d.so.3.1.0;/usr/local/lib/libopencv_tracking.so.3.1.0;/usr/local/lib/libopencv_text.so.3.1.0;/usr/local/lib/libopencv_surface_matching.so.3.1.0;/usr/local/lib/libopencv_structured_light.so.3.1.0;/usr/local/lib/libopencv_stereo.so.3.1.0;/usr/local/lib/libopencv_sfm.so.3.1.0;/usr/local/lib/libopencv_saliency.so.3.1.0;/usr/local/lib/libopencv_rgbd.so.3.1.0;/usr/local/lib/libopencv_reg.so.3.1.0;/usr/local/lib/libopencv_plot.so.3.1.0;/usr/local/lib/libopencv_optflow.so.3.1.0;/usr/local/lib/libopencv_line_descriptor.so.3.1.0;/usr/local/lib/libopencv_fuzzy.so.3.1.0;/usr/local/lib/libopencv_face.so.3.1.0;/usr/local/lib/libopencv_dpm.so.3.1.0;/usr/local/lib/libopencv_dnn.so.3.1.0;/usr/local/lib/libopencv_datasets.so.3.1.0;/usr/local/lib/libopencv_ccalib.so.3.1.0;/usr/local/lib/libopencv_bioinspired.so.3.1.0;/usr/local/lib/libopencv_bgsegm.so.3.1.0;/usr/local/lib/libopencv_aruco.so.3.1.0;/usr/local/lib/libopencv_videostab.so.3.1.0;/usr/local/lib/libopencv_videoio.so.3.1.0;/usr/local/lib/libopencv_video.so.3.1.0;/usr/local/lib/libopencv_superres.so.3.1.0;/usr/local/lib/libopencv_stitching.so.3.1.0;/usr/local/lib/libopencv_shape.so.3.1.0;/usr/local/lib/libopencv_photo.so.3.1.0;/usr/local/lib/libopencv_objdetect.so.3.1.0;/usr/local/lib/libopencv_ml.so.3.1.0;/usr/local/lib/libopencv_imgproc.so.3.1.0;/usr/local/lib/libopencv_imgcodecs.so.3.1.0;/usr/local/lib/libopencv_highgui.so.3.1.0;/usr/local/lib/libopencv_flann.so.3.1.0;/usr/local/lib/libopencv_features2d.so.3.1.0;/usr/local/lib/libopencv_core.so.3.1.0;/usr/local/lib/libopencv_calib3d.so.3.1.0;optimized;/usr/local/lib/libpcl_common.so;debug;/usr/local/lib/libpcl_common.so;optimized;/usr/local/lib/libpcl_octree.so;debug;/usr/local/lib/libpcl_octree.so;optimized;/usr/local/lib/libpcl_io.so;debug;/usr/local/lib/libpcl_io.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/libOpenNI.so")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND stereo_assignment_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND stereo_assignment_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND stereo_assignment_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/arktheshadow/ARK-Linux/Programming/ROS/CMU-PS/cmu-airlab-ps/stereo_catkin_ws_airlab/install/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(stereo_assignment_LIBRARY_DIRS ${lib_path})
      list(APPEND stereo_assignment_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'stereo_assignment'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND stereo_assignment_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(stereo_assignment_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${stereo_assignment_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 stereo_assignment_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${stereo_assignment_dep}_FOUND)
      find_package(${stereo_assignment_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${stereo_assignment_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(stereo_assignment_INCLUDE_DIRS ${${stereo_assignment_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(stereo_assignment_LIBRARIES ${stereo_assignment_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${stereo_assignment_dep}_LIBRARIES})
  _list_append_deduplicate(stereo_assignment_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(stereo_assignment_LIBRARIES ${stereo_assignment_LIBRARIES})

  _list_append_unique(stereo_assignment_LIBRARY_DIRS ${${stereo_assignment_dep}_LIBRARY_DIRS})
  list(APPEND stereo_assignment_EXPORTED_TARGETS ${${stereo_assignment_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${stereo_assignment_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
