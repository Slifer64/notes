

if(MATLAB_FIND_REQUIRED)
    set(MSG_STATUS "FATAL_ERROR")
else()
    set(MSG_STATUS "WARNING")
endif()

# ======== MATLAB root =======
set(MATLAB_ROOT "/home/slifer/MATLAB/R2018b" CACHE PATH "MATLAB root directory")  # TODO: Change to your path!!

# Check if the root-path exists
if(NOT EXISTS "${MATLAB_ROOT}")
    message(${MSG_STATUS} "\033[1;31mERROR:\033[0m The path '${MATLAB_ROOT}' does not exist!")
    return()
endif()

# ======== MATLAB include path =======

set(MATLAB_INCLUDE_PATH "${MATLAB_ROOT}/extern/include")
# Check if the include-path exists
if(NOT EXISTS "${MATLAB_INCLUDE_PATH}")
    message(${MSG_STATUS} "\033[1;31mERROR:\033[0m The path '${MATLAB_INCLUDE_PATH}' does not exist!")
    return()
endif()

# ======== MATLAB libraries =======

set(MATLAB_LIB_PATH "${MATLAB_ROOT}/bin/glnxa64/${library}")
# Check if the library-path exists
if(NOT EXISTS "${MATLAB_LIB_PATH}")
    message(${MSG_STATUS} "\033[1;31mERROR:\033[0m The path '${MATLAB_LIB_PATH}' does not exist!")
    return()
endif()

set(MATLAB_LIBRARIES "libmat.so" "libmx.so" "libmex.so")
# Check if the matlab libraries exists
foreach(library ${MATLAB_LIBRARIES})
    if (NOT EXISTS "${MATLAB_LIB_PATH}/${library}")
        message(${MSG_STATUS} "\033[1;31mERROR:\033[0m Could not find required MATLAB library '${library}' in '${MATLAB_LIB_PATH}'")
        return()
    endif()
endforeach()
