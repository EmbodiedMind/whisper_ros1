cmake_minimum_required(VERSION 3.5)
project(whisper_ros1)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

# Set paths for whisper library
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

## Find CUDA (if needed)
find_package(CUDA REQUIRED)

# Enable GGML_CUDA flag
set(GGML_CUDA ON)  # 启用 CUDA 支持

# Find whisper library (if needed)
find_library(WHISPER_LIB whisper ${WHISPER_LIBRARY_PATH} REQUIRED)
find_path(WHISPER_INCLUDE_DIR whisper.h)

# Define whisper library include directories and libraries
set(WHISPER_LIB_INCLUDE_DIRS ${WHISPER_INCLUDE_DIR})
set(WHISPER_LIB_LIBRARIES ${WHISPER_LIB})

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
# Build the whisper library with CUDA support
add_library(whisper
            lib/whisper.cpp/include/whisper.h
            lib/whisper.cpp/src/whisper.cpp  # 手动指定路径
)

# Enable CUDA for whisper if necessary
if(GGML_CUDA)
  target_compile_definitions(whisper PRIVATE GGML_CUDA=1)  # 定义 GGML_CUDA 宏
  target_link_libraries(whisper PRIVATE ggml ${CUDA_LIBRARIES})  # 链接 ggml 和 CUDA 库
endif()

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

# 链接 whisper 库到 common
target_link_libraries(${TARGET} PRIVATE whisper)
set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Build the common-ros library
set(TARGET common-ros)
add_library(${TARGET} STATIC
    include/common-ros.h
    src/common-ros.cpp
    include/rosaudio_controller.h
    src/rosaudio_controller.cpp
)

target_link_libraries(${TARGET} PRIVATE whisper ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})
set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Build the roswhisper-stream executable
set(TARGET roswhisper-stream)
add_executable(${TARGET} src/ros_stream.cpp)

target_link_libraries(${TARGET} PRIVATE ${catkin_LIBRARIES} common common-ros whisper ggml ${CMAKE_THREAD_LIBS_INIT})

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

