cmake_minimum_required(VERSION 2.8.3)
project(thor_math)
find_package(cnr_logger REQUIRED)

find_package(Eigen3 REQUIRED)


add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/thor_math.cpp
  )
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++20 -funroll-loops -Wall -Ofast)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  )
add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})


#add_executable(test_evolution_matrix src/test_evolution_matrix.cpp)
#add_dependencies(test_evolution_matrix ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_evolution_matrix
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  )


add_executable(test_thor_math src/test_thor_math.cpp)
target_link_libraries(test_thor_math
  PUBLIC
  "${PROJECT_NAME}::${PROJECT_NAME}"
  )

#add_executable(test_thor_math2 src/test_thor_math2.cpp)
#add_dependencies(test_thor_math2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_thor_math2
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  )

#add_executable(test_thor_clik src/test_thor_clik.cpp)
#add_dependencies(test_thor_clik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_thor_clik
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  )

