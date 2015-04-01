cmake_minimum_required(VERSION 2.8.3)
project(ros_msgs_sync)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/sync_image_handler.cpp
  src/sync_image_transport_handler.cpp
  include/${PROJECT_NAME}/impl/sync_impl_transport_handler.cpp
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} 
)

## Declare a cpp executable

add_executable(img_trp_sync_save sample/sample_sync_save.cpp)
target_link_libraries(img_trp_sync_save
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

add_executable(img_sync_disp sample/sample_sync_disp.cpp)
target_link_libraries(img_sync_disp
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

add_executable(ptcld_sync_save sample/sample_sync_pointcloud.cpp)
target_link_libraries(ptcld_sync_save
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS img_trp_sync_save img_sync_disp ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
