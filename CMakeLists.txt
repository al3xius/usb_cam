cmake_minimum_required(VERSION 2.8.12)
project(usb_cam_auto_exposure)

# DEPENCENCIES
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  cv_bridge 
  image_transport 
  roscpp 
  std_msgs 
  std_srvs 
  sensor_msgs 
  camera_info_manager 
  dynamic_reconfigure
)

# Find OpenCV
find_package(OpenCV REQUIRED)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/CameraSettings.cfg
  cfg/ControllerSettings.cfg
)



###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usb_cam_auto_exposure vo_autoexpose
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs camera_info_manager dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)


## Build the USB camera library
add_library(${PROJECT_NAME} 
  src/usb_cam.cpp
  src/vo_autoexpose.cpp 
  include/vo_autoexpose/vo_autoexpose.h 
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS} 
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


#############
## Install ##
#############

# ## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# ## Copy launch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch"
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
# )
