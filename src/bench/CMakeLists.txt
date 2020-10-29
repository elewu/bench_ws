CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(bench)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             nav_msgs
             tf
             cv_bridge)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES("../ORB_SLAM3/")
INCLUDE_DIRECTORIES("../ORB_SLAM3/include")
INCLUDE_DIRECTORIES("../ORB_SLAM3/include/CameraModels")
LINK_DIRECTORIES("../ORB_SLAM3/lib")

CATKIN_PACKAGE(CATKIN_DEPENDS pangolin)

SET(DEPS ${catkin_LIBRARIES}
         ${OpenCV_LIBS}
         ${Pangolin_LIBRARIES}
         ORB_SLAM3)

ADD_EXECUTABLE(orbslam3_mono src/orbslam3/ros_mono.cc)
TARGET_LINK_LIBRARIES(orbslam3_mono ${DEPS})

ADD_EXECUTABLE(orbslam3_mono_inertial src/orbslam3/ros_mono_inertial.cc)
TARGET_LINK_LIBRARIES(orbslam3_mono_inertial ${DEPS})

ADD_EXECUTABLE(orbslam3_rgbd src/orbslam3/ros_rgbd.cc)
TARGET_LINK_LIBRARIES(orbslam3_rgbd ${DEPS})

ADD_EXECUTABLE(orbslam3_stereo src/orbslam3/ros_stereo.cc)
TARGET_LINK_LIBRARIES(orbslam3_stereo ${DEPS})

ADD_EXECUTABLE(orbslam3_stereo_inertial src/orbslam3/ros_stereo_inertial.cc)
TARGET_LINK_LIBRARIES(orbslam3_stereo_inertial ${DEPS})

install(TARGETS
  orbslam3_mono
  orbslam3_mono_inertial
  orbslam3_rgbd
  orbslam3_stereo
  orbslam3_stereo_inertial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
