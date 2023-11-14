

## `ament_package`
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_project)

# ...

ament_package()
```
Registers the package and installs config files for CMake so that it can be found by other packages using `find_package`. Should be the last call in your `CMakeLists.txt`.

## Warnings
Recommended to at least cover the following warning levels:
```cmake
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

## `find_package`
```cmake
SET(PKG_DEPENDENCIES
    rclcpp
    <some_package>
    <another_package>
)

foreach(package ${PKG_DEPENDENCIES})
  find_package(${package} REQUIRED)
endforeach()
```

It should never be necessary to find_package a library that is not explicitly needed but is a dependency of another dependency that is explicitly needed. If that is the case, file a bug against the corresponding package.


## Include directories
```cmake
#include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS})
```
Only local directories and dependencies that are not ament packages need to be manually included. \
Better alternative: specify include directories for each target individually, rather than including all the directories for all targets:
```cmake
target_include_directories(<target_name> PUBLIC include ${Boost_INCLUDE_DIRS})
```

## Targets (executable / libraries)

### Library
```cmake
add_library(<lib_name> <src_files>)
target_include_directories(<lib_name>
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
```
The latter adds all files in the folder `${CMAKE_CURRENT_SOURCE_DIR}/include` to the public interface during build time and all files in the `include` folder (relative to `${CMAKE_INSTALL_DIR}`) when being installed. \
**Tip**: Put all headers which should be usable by clients of this library (and therefore must be installed) into a subdirectory of the include folder named like the package, while all other files (.c/.cpp and header files which should not be exported) are inside the src folder.

### Executable
```cmake
add_executable(<exec_name> <src_files>)
```
The executable may also have to be linked with any libraries created in this package by using `target_link_libraries`.

## Linking to dependencies
### 1st way (recommended for ament packages).
```cmake
ament_target_dependencies(<lib_name> PUBLIC rclcpp Eigen3)
```
Automatically handles both the include directories defined in `_INCLUDE_DIRS` and linking libraries defined in `_LIBRARIES`. Can also be used with non-ament packages.

### 2nd way (for non ament packages)
Link with non-ament packages, such as system dependencies like Boost, or a library being built in the same `CMakeLists.txt`.
```cmake
target_link_libraries(<lib_name> PUBLIC armadillo Eigen3::Eigen)
```
Includes necessary headers, libraries and their dependencies. \
If available opt for namespaced targets.


## Install

### Library

```cmake
# headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# targets
install(
  TARGETS <lib_name1> <lib_name2>
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# necessary to allow your library’s clients to use the target_link_libraries(client PRIVATE my_library::my_library) syntax
# If the export set includes a library, add the option HAS_LIBRARY_TARGET to ament_export_targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# exports dependencies to downstream packages. Necessary so that the user of the library does not have to call find_package for those dependencies, too
ament_export_dependencies(rclcpp <other_dependency>)

# Optional. Use only if the downstream projects can’t or don’t want to use CMake target based dependencies

# marks the directory of the exported include directories
# (done by the `HAS_LIBRARY_TARGET` in `ament_export_targets`
ament_export_include_directories("include/${PROJECT_NAME}")

# marks the location of the installed library
ament_export_libraries(my_library)
```

### Executable

```cmake
install(TARGETS
    <exec_name1>
    <exec_name2>
    DESTINATION lib/${PROJECT_NAME})
```