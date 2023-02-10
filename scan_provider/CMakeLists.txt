add_executable (scan_provider
	"${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
)

target_sources(scan_provider PRIVATE
	"${CMAKE_CURRENT_SOURCE_DIR}/src/LidarConnection.cpp"
 "include/definitions.h")

target_include_directories(scan_provider PRIVATE
	"${CMAKE_CURRENT_SOURCE_DIR}/src"
	"${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(scan_provider LINK_PUBLIC sl_lidar_sdk)
target_link_libraries(scan_provider LINK_PUBLIC cpr)
