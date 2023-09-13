cmake_minimum_required(VERSION 3.0.2)
project(camera_node)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES ResizedImage.msg
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(camera_node src/camera_node.cpp)
add_dependencies(camera_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})




# Mark executables and/or libraries for installation
install(TARGETS camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
