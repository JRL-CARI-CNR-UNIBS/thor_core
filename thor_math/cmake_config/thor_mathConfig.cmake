include(CMakeFindDependencyMacro)


## insert here the dependencies
find_dependency(Eigen3 REQUIRED Core Dense Geometry)
find_dependency(rdyn_core REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/thor_mathTargets.cmake")

check_required_components(${PROJECT_NAME})