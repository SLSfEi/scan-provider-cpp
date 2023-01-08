set(lib_root ${CMAKE_CURRENT_SOURCE_DIR})

# rplidar lib
set(sl_lidar_sdk_srcs
	"${lib_root}/src/sl_lidar_driver.cpp"
	"${lib_root}/src/hal/thread.cpp"
	"${lib_root}/src/sl_crc.cpp"
	"${lib_root}/src/sl_serial_channel.cpp"
	"${lib_root}/src/sl_tcp_channel.cpp"
	"${lib_root}/src/sl_udp_channel.cpp"
)
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	list(APPEND sl_lidar_sdk_srcs
	"${lib_root}/src/arch/linux/net_serial.cpp"
	"${lib_root}/src/arch/linux/net_socket.cpp"
	"${lib_root}/src/arch/linux/timer.cpp"
	)
endif()
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	list(APPEND sl_lidar_sdk_srcs
	"${lib_root}/src/arch/macOS/net_serial.cpp"
	"${lib_root}/src/arch/maxOS/net_socket.cpp"
	"${lib_root}/src/arch/macOS/timer.cpp"
	)
endif()
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	list(APPEND sl_lidar_sdk_srcs
	"${lib_root}/src/arch/win32/net_serial.cpp"
	"${lib_root}/src/arch/win32/net_socket.cpp"
	"${lib_root}/src/arch/win32/timer.cpp"
	)
endif()
message(STATUS "Building sl_lidar_sdk for ${CMAKE_SYSTEM_NAME}")

add_library(sl_lidar_sdk STATIC ${sl_lidar_sdk_srcs})
target_link_libraries(sl_lidar_sdk "$<$<CXX_COMPILER_ID:GNU>:-pthread>$<$<CXX_COMPILER_ID:Clang>:-pthreads>")
target_include_directories(sl_lidar_sdk PUBLIC
	"${lib_root}/src"
	"${lib_root}/include"
)
