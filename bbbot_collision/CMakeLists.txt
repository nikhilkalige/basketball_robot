cmake_minimum_required(VERSION 2.8.3)
project(bbbot_collision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
    REQUIRED
    COMPONENTS
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        std_msgs
        trajectory_msgs
        cmake_modules
        message_generation
)

add_service_files(
    FILES
    CollisionCheck.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
    CATKIN_DEPENDS
        std_msgs
        trajectory_msgs
        # moveit_core
        # moveit_ros_planning_interface
)

# find_package(Eigen REQUIRED)

###########
## Build ##
###########

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()


include_directories(include
    SYSTEM
    #${Boost_INCLUDE_DIR}
    #${EIGEN_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SOURCES
    src/bbbot_collision.cpp
)

add_executable(bbbot_collision ${${PROJECT_NAME}_SOURCES})

add_dependencies(bbbot_collision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bbbot_collision
    ${catkin_LIBRARIES}
    # ${Boost_LIBRARIES}
)

install(TARGETS bbbot_collision DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
