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
file(GLOB_RECURSE CONFIG_FILES "config/*")
# file(COPY ${CONFIG_FILES} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
configure_file(${CONFIG_FILES} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COPYONLY)

target_link_libraries(scan_provider LINK_PUBLIC sl_lidar_sdk)
target_link_libraries(scan_provider LINK_PUBLIC cpr)
