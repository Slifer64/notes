
# Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release # or Debug
make
```

Another way:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target install
```
This allows to run cmake from the source directory. It will create the `build` directory and the `--build` tag specifys the path where the build will take place.

# Set c++ standard
E.g. to set `c++17`:
```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# or
set(CMAKE_CXX_STANDARD 17)
```

# Include Qt
```cmake
set(CMAKE_AUTOMOC ON) # Tell CMake to run moc when needed.
set(CMAKE_AUTOUIC ON) # Tell CMake to run uic when needed.
set(CMAKE_AUTORCC ON) # Tell CMake to run rcc when needed
# ...
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
set(Qt5_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Gui)
# ...
add_executable(${exec_name} ${SRC_LIST})

target_include_directories(${exec_name} 
  PRIVATE # or PUBLIC
    ${Qt5Widgets_INCLUDES}
    # other includes
)

target_compile_definitions(${exec_name} PRIVATE ${Qt5Widgets_DEFINITIONS})

target_link_libraries(${exec_name} 
  ${Qt5_LIBRARIES}
  # other libs
)
```

# Include Eigen
```cmake
find_package(Eigen3 REQUIRED)
# ...
add_executable(${exec_name} ${SRC_LIST})

target_link_libraries(${exec_name} 
  Eigen3::Eigen
  # other libs
)
```

# Include OpenCV
```cmake
find_package(OpenCV REQUIRED)
# ...
add_executable(${exec_name} ${SRC_LIST})

target_include_directories(${exec_name} 
  PRIVATE # or PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    # other includes
)

target_link_libraries(${exec_name} 
  ${OpenCV_LIBS}
  # other libs
)
```

# Enable useful warnings
```cmake
add_compile_options(-Wall -Wextra -Wpedantic)
```

# Add options
```cmake
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_EXAMPLES "Build example executables" ON)
# ...
if (BUILD_EXAMPLES)
  # ...
endif()
```
And to set options:
```bash
cmake <path_to_CMakeLists.txt> -DBUILD_SHARED_LIBS=OFF -BUILD_EXAMPLES=ON
```
