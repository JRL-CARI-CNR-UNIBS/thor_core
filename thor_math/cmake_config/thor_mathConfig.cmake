include(CMakeFindDependencyMacro)

# Ensure the pinocchio target exists before loading our exported targets:
find_dependency(Eigen3 REQUIRED Core Dense Geometry)
find_dependency(pinocchio REQUIRED) # or: find_dependency(pinocchio CONFIG REQUIRED)
find_dependency(rdyn_core REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/thor_mathTargets.cmake")

check_required_components(${PROJECT_NAME})
