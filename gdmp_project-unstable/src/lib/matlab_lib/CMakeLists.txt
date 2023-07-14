cmake_minimum_required(VERSION 3.0)

project(matlab_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_compile_options(-fpermissive)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# =========  create library .so  ========

find_package(matlab REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${MATLAB_INCLUDE_PATH}
)

include_directories(
    include
    ${MATLAB_INCLUDE_PATH}
    ${catkin_INCLUDE_DIRS}
    "/usr/include/eigen3"
)

link_directories(${MATLAB_LIB_PATH})

set(LIBRARY_NAME ${PROJECT_NAME})
set(SOURCE_FILES src/utils.cpp)
set(TARGET_LINK_LIBS ${MATLAB_LIBRARIES} armadillo)

add_library(${LIBRARY_NAME} ${SOURCE_FILES})
add_dependencies(${LIBRARY_NAME}
  ${${LIBRARY_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${LIBRARY_NAME} ${TARGET_LINK_LIBS})

# =======  test-example  ==========
add_executable(${PROJECT_NAME}_test ${SOURCE_FILES} src/test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${TARGET_LINK_LIBS})
