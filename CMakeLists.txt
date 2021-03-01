cmake_minimum_required(VERSION 2.8.3)
project(RRT_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  
  geometry_msgs
  std_msgs
  visualization_msgs
)

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_planner image_loader
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${BULLET_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(rrt_planner src/rrt.cpp)
add_dependencies(rrt_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_planner
  ${catkin_LIBRARIES}
)

add_library(image_loader src/image_loader.cpp)
add_dependencies(image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_loader
  ${BULLET_LIBRARIES}
  ${catkin_LIBRARIES}
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
)

add_executable(planner src/planner.cpp)
add_dependencies(planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner
  rrt_planner
  ${catkin_LIBRARIES}
)

add_executable(map_server src/map_server.cpp)
add_dependencies(map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server
    image_loader
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)