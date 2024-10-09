# - Try to find acados
# Once done, this will define
#  ACADOS_FOUND        - True if acados was found
#  ACADOS_INCLUDE_DIRS - Where to find the acados headers
#  ACADOS_LIBRARIES    - The libraries to link against

# Use environment variables to guide search locations
set(ACADOS_INCLUDE_SEARCH_PATHS
  "/usr/include"
  "/usr/local/include"
  "/opt/acados/include"
  "$ENV{ACADOS_HOME}"
  "$ENV{ACADOS_HOME}/include"
  "$ENV{ACADOS_SOURCE_DIR}"
  "$ENV{ACADOS_SOURCE_DIR}/include"
)

set(ACADOS_LIB_SEARCH_PATHS
  "/usr/lib"
  "/usr/local/lib"
  "/opt/acados/lib"
  "$ENV{ACADOS_HOME}/lib"
  "$ENV{ACADOS_SOURCE_DIR}/lib"
)

# Find acados include directory
find_path(ACADOS_INCLUDE_DIR_BASE NAMES acados PATHS ${ACADOS_INCLUDE_SEARCH_PATHS})

# Find acados library
find_library(ACADOS_LIB NAMES acados PATHS ${ACADOS_LIB_SEARCH_PATHS})

# If the library and include directory base are found, define ACADOS_FOUND
if(ACADOS_LIB AND ACADOS_INCLUDE_DIR_BASE)
    set(ACADOS_FOUND TRUE)

    # Set the directories based on your structure
    set(ACADOS_INCLUDE_DIRS 
        ${ACADOS_INCLUDE_DIR_BASE}
        ${ACADOS_INCLUDE_DIR_BASE}/acados
        ${ACADOS_INCLUDE_DIR_BASE}/blasfeo/include
        ${ACADOS_INCLUDE_DIR_BASE}/hpipm/include
        ${ACADOS_INCLUDE_DIR_BASE}/qpOASES_e
    )
    set(ACADOS_LIBRARIES ${ACADOS_LIB})
else()
    set(ACADOS_FOUND FALSE)
endif()

# Provide a message for debugging
if(ACADOS_FOUND)
    message(STATUS "Found acados: ${ACADOS_LIBRARY}")
    message(STATUS "acados include dir: ${ACADOS_INCLUDE_DIR}")
else()
    message(WARNING "acados not found")
endif()

# Make the library importable by CMake
mark_as_advanced(ACADOS_LIBRARY ACADOS_INCLUDE_DIR)
