cmake_minimum_required(VERSION 3.16.3)
project(thor_math)

# Dependencies
find_package(Eigen3 REQUIRED  COMPONENTS Core Dense Geometry)
find_package(rdyn_core REQUIRED)


# Libraries (SHARED: dynamic .so/.dll)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/thor_math.cpp # list of source files
  )

target_link_libraries(${PROJECT_NAME}  PUBLIC
  Eigen3::Eigen
  rdyn_core::rdyn_core
)

target_compile_options(${PROJECT_NAME} PUBLIC -std=c++20 -funroll-loops -Ofast)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(thor_math::thor_math ALIAS thor_math)


#add_executable(test_evolution_matrix src/test_evolution_matrix.cpp)
#add_dependencies(test_evolution_matrix ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_evolution_matrix
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  )


#add_executable(test_thor_math src/test_thor_math.cpp)
#target_link_libraries(test_thor_math
#  PUBLIC
#  "${PROJECT_NAME}::${PROJECT_NAME}"
#  )

add_executable(test_thor_math2
  src/test_thor_math2.cpp)

target_link_libraries(test_thor_math2
  PUBLIC
  thor_math::thor_math
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


#################
## END - Build ##
#################

### #############
### ## Install ##
### #############

# Installation instructions for the thor_math library and its dependencies

# Install header files from the include/thor_math directory to the include directory
install(DIRECTORY include/thor_math
  DESTINATION include)

# Install the thor_math library target to appropriate system locations:
# - Archive files (static libraries) to lib
# - Shared libraries to lib
# - Executable binaries to bin
install(
  TARGETS thor_math test_thor_math2
  EXPORT thor_mathTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export thor_mathTargets to enable other CMake projects to import this library
# thor_mathTargets.cmake will be generated and placed in share/thor_math/cmake
install(EXPORT thor_mathTargets
  DESTINATION "share/thor_math/cmake"
  NAMESPACE thor_math::
  FILE thor_mathTargets.cmake
)

# Generate a version file (thor_mathConfigVersion.cmake) to enable version checking
# during library import and specify compatibility with any newer version
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/thor_mathConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

# Install the configuration files to allow importing thor_math in CMake projects
install(FILES
  ${CMAKE_SOURCE_DIR}/cmake_config/thor_mathConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/thor_mathConfigVersion.cmake
  DESTINATION "share/thor_math/cmake")

# Custom uninstall target to remove installed files
# The uninstall command uses install_manifest.txt to track and delete installed files
# If no files are listed in install_manifest.txt, an informative message is printed
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)
