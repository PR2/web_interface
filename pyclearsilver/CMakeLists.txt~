cmake_minimum_required(VERSION 2.8.3)
project(pyclearsilver)
find_package(catkin REQUIRED)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roslib rospy rosservice rosjson std_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY src DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
