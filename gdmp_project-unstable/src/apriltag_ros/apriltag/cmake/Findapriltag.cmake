set(apriltag_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/apriltag/include/
)

set(apriltag_LIBDIR 
    ${CMAKE_CURRENT_SOURCE_DIR}/apriltag/lib/
)

set(apriltag_LIBRARIES 
    ${CMAKE_CURRENT_SOURCE_DIR}/apriltag/lib/libapriltag.so
)


if(apriltag_INCLUDE_DIRS AND apriltag_LIBDIR)

  set( apriltag_FOUND true )

endif()

if(apriltag_FOUND)
  message(STATUS "Found apriltag Library: ${apriltag_LIBRARIES}")
else()
  message(FATAL_ERROR "Could not find apriltag Library")
endif()