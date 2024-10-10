cmake_minimum_required(VERSION 3.5)
project(nmpc_nav_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  geometry_msgs
  itrci_nav
  parametric_trajectories_common
)

# Find additional libraries
find_package(Eigen3 REQUIRED)

# Add the cmake/ directory to CMake's module search path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(acados REQUIRED)

# Acados installation path and c_generated code path
set(omni4amr_model ${PROJECT_SOURCE_DIR}/scripts/omni4/c_generated_code)
set(diff2amr_model ${PROJECT_SOURCE_DIR}/scripts/diff/c_generated_code)
set(tric3amr_model ${PROJECT_SOURCE_DIR}/scripts/tric/c_generated_code)

# Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    geometry_msgs
    itrci_nav
    parametric_trajectories_common
  DEPENDS 
    EIGEN3
    ACADOS
)

# Build
# - additional header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ACADOS_INCLUDE_DIRS}
  ${omni4amr_model}
  ${diff2amr_model} 
  ${tric3amr_model} 
)
link_directories(${omni4amr_model})
link_directories(${diff2amr_model})
link_directories(${tric3amr_model})

# - declare a C++ library
# add_executable(${PROJECT_NAME}
#   src/${PROJECT_NAME}/NMPCNavControlROS.cpp
#   src/${PROJECT_NAME}/NMPCNavControl.cpp
#   src/${PROJECT_NAME}/NMPCNavControlOmni4.cpp
#   src/${PROJECT_NAME}/NMPCNavControlDiff.cpp
#   src/${PROJECT_NAME}/NMPCNavControlTric.cpp
#   src/${PROJECT_NAME}/PathDiscretizer.cpp
# )

# - declare a C++ executable
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}/NMPCNavControlROS.cpp
  src/${PROJECT_NAME}/NMPCNavControl.cpp
  src/${PROJECT_NAME}/NMPCNavControlOmni4.cpp
  src/${PROJECT_NAME}/NMPCNavControlDiff.cpp
  src/${PROJECT_NAME}/NMPCNavControlTric.cpp
  src/${PROJECT_NAME}/PathDiscretizer.cpp
  src/main.cpp
)

# - target dependencies
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# add_dependencies(${PROJECT_NAME}      # for the library
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# - specify linking libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${ACADOS_LIBRARIES}
  ${omni4amr_model}/libacados_ocp_solver_omni4amr.so
  ${diff2amr_model}/libacados_ocp_solver_diff2amr.so
  ${tric3amr_model}/libacados_ocp_solver_tric3amr.so
)

# # - installation rules
# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # install(TARGETS ${PROJECT_NAME}       # for the library
# #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# # )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(
#   DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )