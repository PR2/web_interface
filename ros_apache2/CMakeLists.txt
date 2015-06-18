cmake_minimum_required(VERSION 2.8.3)
project(ros_apache2)
find_package(catkin REQUIRED COMPONENTS webui)

catkin_package(
    DEPENDS webui
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY scripts conf www DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/ros_apache2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

