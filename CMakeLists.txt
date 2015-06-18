cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROSPACK_MAKEDIST true)

# Append to CPACK_SOURCE_IGNORE_FILES a semicolon-separated list of
# directories (or patterns, but directories should suffice) that should
# be excluded from the distro.  This is not the place to put things that
# should be ignored everywhere, like "build" directories; that happens in
# rosbuild/rosbuild.cmake.  Here should be listed packages that aren't
# ready for inclusion in a distro.
#
# This list is combined with the list in rosbuild/rosbuild.cmake.  Note
# that CMake 2.6 may be required to ensure that the two lists are combined
# properly.  CMake 2.4 seems to have unpredictable scoping rules for such
# variables.
#list(APPEND CPACK_SOURCE_IGNORE_FILES /core/experimental)

# IMPORTANT: the next release after 0.3.21 will be 0.4.0, not 0.3.22!  
# If you want to make an 0.3.x release (e.g., for inclusion in boxturtle),
# then you should be working from the 0.3 branch
# (https://code.ros.org/svn/wg-ros-pkg/stacks/web_interface/branches/web_interface-0.3)
rosbuild_make_distribution(0.5.0)
