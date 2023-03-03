cmake_minimum_required (VERSION 3.8)

project ("smart_lidar")
set(cpp_standard MSVC11)
set(cpp_standard_required true)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out/bin") # .exe and .dll
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out/lib") # .so and .dylib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out/lib") # .lib and .a


add_subdirectory(scan_provider)
add_subdirectory(lib/rplidar)
add_subdirectory(lib/cpr)