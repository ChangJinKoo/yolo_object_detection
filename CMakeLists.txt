cmake_minimum_required(VERSION 2.8.3)
project(yolo_object_detection)

#add_compile_options(--std=c++11)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(ENABLE_DRAFT ON)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    message_generation
)

find_package(OpenCV 4.4.0 REQUIRED)

add_message_files(
    FILES
    bounding_box.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(DARKNET_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/darknet/include)
set(DARKNET_LIBRARY ${PROJECT_SOURCE_DIR}/darknet/libdarknet.so)

catkin_package(
#    CATKIN_DEPENDS roscpp cv_bridge image_transport  sensor_msgs
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${DARKNET_INCLUDE_DIR}
)
    
add_executable(${PROJECT_NAME}
     src/main.cpp 
     src/yolo_object_detection_node.cpp
     )
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${DARKNET_LIBRARY}
    )
