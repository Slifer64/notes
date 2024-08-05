<!-- TABLE OF CONTENTS -->
<h1 align="left">TOC</h1>
<details>
  <summary>Table of Contents</summary>
  <ul>
    <li><a href="#build">Built</a></li>
    <li><a href="#useful-warnings">Useful warnings</a></li>
    <li><a href="#set-c-standard">C++ standard</a></li>
    <li>
      <a href="#include-libraries">Useful libraries</a>
      <ul>
        <li><a href="#qt">Qt</a></li>
        <li><a href="#eigen">Eigen</a></li>
        <li><a href="#opencv">OpenCV</a></li>
      </ul>
    </li>
  </ul>
  <li><a href="#add-options">Add cmake options</a></li>
  <li><a href="#add-to-default-library-paths">Add to library path</a></li>
  <li><a href="#colored-message">Colored message</a></li>
</details>

---

# Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release # or Debug
make
```

Another way:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=./install
cmake --build build --target install
```
This allows to run cmake from the source directory. It will create the `build` directory and the `--build` tag specifys the path where the build will take place.

# Useful warnings
```cmake
add_compile_options(-Wall -Wextra -Wpedantic)
```

# Set c++ standard
E.g. to set `c++17`:
```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# or
set(CMAKE_CXX_STANDARD 17)
```

# Include libraries

## Qt
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

## Eigen
```cmake
find_package(Eigen3 REQUIRED)
# ...
add_executable(${exec_name} ${SRC_LIST})

target_link_libraries(${exec_name} 
  Eigen3::Eigen
  # other libs
)
```

## OpenCV
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

# Add to default library paths
`LB_LIBRARY_PATH` contains defaults paths, from where the linked `*.so` files are searched. \
To add an extra path, e.g. `/usr/local/lib`, append it to `LB_LIBRARY_PATH` and add it to `.bashrc`: 
```bash
echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
```

# Colored message
```cmake
string(ASCII 27 ESC)
message("${ESC}[34m" "This is a blue message" "${ESC}[0m")
```