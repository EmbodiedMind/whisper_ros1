cmake_minimum_required(VERSION 3.5)
project(whisper_ros1)

## Compile as C++11, supported in ROS Kinetic and newer

set(WHISPER_LIB_PATH "lib/whisper.cpp")
set(WHISPER_INCLUDE_DIR "lib/whisper.cpp/include")


add_subdirectory("lib/whisper.cpp/ggml")


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${WHISPER_LIB_PATH}  # whisper 的头文件路径
  ${WHISPER_INCLUDE_DIR}
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Threads REQUIRED)

find_library(WHISPER_LIB whisper ${WHISPER_LIBRARY_PATH} REQUIRED)
find_path(WHISPER_INCLUDE_DIR whisper.h)

# if(NOT WHISPER_LIB OR NOT WHISPER_INCLUDE_DIR)
#   message(FATAL_ERROR "Could not find whisper library or include path")
# endif()

# 定义 WHISPER_LIB_INCLUDE_DIRS 和 WHISPER_LIB_LIBRARIES
set(WHISPER_LIB_INCLUDE_DIRS ${WHISPER_INCLUDE_DIR})
set(WHISPER_LIB_LIBRARIES ${WHISPER_LIB})

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  lib/whisper.cpp
  ${WHISPER_INCLUDE_DIR}
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include ${WHISPER_INCLUDE_DIR}
  LIBRARIES common common-ros whisper
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS WHISPER_LIB
)

###########
## Build ##
###########
# 将 whisper 源文件编译成库

add_library(whisper
            lib/whisper.cpp/include/whisper.h
            lib/whisper.cpp/src/whisper.cpp  # 手动指定路径
            )

# Build the common library
set(TARGET common)
add_library(${TARGET} STATIC
    include/common.h
    src/common.cpp
    include/common-ggml.h
    src/common-ggml.cpp
    include/grammar-parser.h
    src/grammar-parser.cpp
)

target_link_libraries(${TARGET} PRIVATE whisper )
set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(whisper 
  ggml
)



# Build the common-ros library
set(TARGET common-ros)
add_library(${TARGET} STATIC
    include/common-ros.h
    src/common-ros.cpp
    include/rosaudio_controller.h
    src/rosaudio_controller.cpp

)

target_link_libraries(${TARGET} PRIVATE whisper ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES} )
set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Build the roswhisper-stream executable
set(TARGET roswhisper-stream)
add_executable(${TARGET} src/ros_stream.cpp)

target_link_libraries(${TARGET} PRIVATE  ${catkin_LIBRARIES} common common-ros whisper ggml ${CMAKE_THREAD_LIBS_INIT})

###########
## Install ##
###########
install(TARGETS common common-ros roswhisper-stream
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)