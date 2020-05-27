cmake_minimum_required(VERSION 2.8.3)
project(basalt_ros1)

if(NOT (DEFINED ENV{ROS_VERSION}))
   message(ERROR, "ROS_VERSION environment variable $ENV{ROS_VERSION} is not set!")
else()
  if(NOT ($ENV{ROS_VERSION} EQUAL "1"))
    unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
    install(TARGETS)   # nothing to install
  else()
  
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CXX_MARCH)
  set(CXX_MARCH native)
endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  nodelet
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  basalt_vio_ros1_msgs
  basalt
)

find_package(Eigen3 REQUIRED QUIET)
find_package(TBB REQUIRED)
find_package(realsense2 QUIET)


# Flags used for CHECK_CXX_SOURCE_COMPILES
set(CMAKE_REQUIRED_FLAGS "-Wno-error")


# save flags passed by user
set(BASALT_PASSED_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"
set(CMAKE_CXX_FLAGS_CIDEBUG  "-O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # CI version with no debug symbols
set(CMAKE_CXX_FLAGS_CIRELWITHDEBINFO "-O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # CI version with no debug symbols

# base set of compile flags
set(BASALT_CXX_FLAGS "-Wall -Wextra -Werror -Wno-error=unused-parameter -ftemplate-backtrace-limit=0")


# OpenMP option and compile flags
option(USE_OPENMP "Use OpenMP (e.g. for parallel computation in Eigen)" ${USE_OPENMP_DEFAULT})
if(USE_OPENMP)
  message(STATUS "OpenMP Enabled")
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -fopenmp")
else()
  message(STATUS "OpenMP Disabled")
endif()


# setup combined compiler flags
set(CMAKE_CXX_FLAGS "-std=c++17 ${BASALT_CXX_FLAGS} -march=${CXX_MARCH} ${BASALT_PASSED_CXX_FLAGS}")

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp nodelet
	)
	
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${basalt_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIR}
)

add_library(basalt_ros1
src/vio_frontend.cpp
src/vio_backend.cpp
src/image_subscriber.cpp
src/imu_subscriber.cpp
src/imu_publisher.cpp
src/optical_flow_publisher.cpp
src/optical_flow_subscriber.cpp
src/vio_publisher.cpp
src/utils.cpp
)

target_link_libraries(basalt_ros1 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

# --- vio_frontend ---
add_executable(vio_frontend_node src/vio_frontend_node.cpp)
target_link_libraries(vio_frontend_node basalt_ros1  ${catkin_LIBRARIES} ${TBB_LIBRARIES})
add_dependencies(vio_frontend_node ${catkin_EXPORTED_TARGETS})

add_library(vio_frontend_nodelet src/vio_frontend_nodelet.cpp)
target_link_libraries(vio_frontend_nodelet basalt_ros1  ${catkin_LIBRARIES} ${TBB_LIBRARIES})
add_dependencies(vio_frontend_nodelet ${catkin_EXPORTED_TARGETS})

# --- vio_backend ---
add_executable(vio_backend_node src/vio_backend_node.cpp)
target_link_libraries(vio_backend_node basalt_ros1  ${catkin_LIBRARIES} ${TBB_LIBRARIES})
add_dependencies(vio_backend_node ${catkin_EXPORTED_TARGETS})

add_library(vio_backend_nodelet src/vio_backend_nodelet.cpp)
target_link_libraries(vio_backend_nodelet basalt_ros1  ${catkin_LIBRARIES} ${TBB_LIBRARIES})
add_dependencies(vio_backend_nodelet ${catkin_EXPORTED_TARGETS})

# --- combined frontend/backend ---

add_executable(vio_node src/vio_node.cpp)
target_link_libraries(vio_node basalt_ros1  ${catkin_LIBRARIES} ${TBB_LIBRARIES})
add_dependencies(vio_node ${catkin_EXPORTED_TARGETS})

add_library(vio_nodelet src/vio_nodelet.cpp)
target_link_libraries(vio_nodelet basalt_ros1  ${catkin_LIBRARIES} ${TBB_LIBRARIES})
add_dependencies(vio_nodelet ${catkin_EXPORTED_TARGETS})

# --- realsense tools ---
if(realsense2_FOUND)
  add_executable(get_calibration src/get_calibration.cpp)
  target_link_libraries(get_calibration
  ${catkin_LIBRARIES} ${TBB_LIBRARIES})
  install(TARGETS get_calibration
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()


# --- install ---

install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS vio_frontend_node vio_backend_node vio_node
                basalt_ros1 vio_frontend_nodelet vio_backend_nodelet vio_nodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


  endif()  # closes   if($ENV{ROS_VERSION} EQUAL "1")

endif()  # closes if (DEFINED ENV{ROS_VERSION})

