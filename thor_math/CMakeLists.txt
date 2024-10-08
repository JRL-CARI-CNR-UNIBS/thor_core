cmake_minimum_required(VERSION 2.8.3)
project(thor_math)

#add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp  eigen_conversions sensor_msgs rosdyn_core
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp eigen_conversions sensor_msgs rosdyn_core
  DEPENDS 
)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/thor_math.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_evolution_matrix src/test_evolution_matrix.cpp)
add_dependencies(test_evolution_matrix ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_evolution_matrix
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(test_thor_math src/test_thor_math.cpp)
add_dependencies(test_thor_math ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_thor_math
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_thor_math2 src/test_thor_math2.cpp)
add_dependencies(test_thor_math2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_thor_math2
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_thor_clik src/test_thor_clik.cpp)
add_dependencies(test_thor_clik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_thor_clik
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

