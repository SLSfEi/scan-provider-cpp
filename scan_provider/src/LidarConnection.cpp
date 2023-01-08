#include "LidarConnection.h"

#include <iostream>
#include <sstream>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"


// Constructor
LidarConnection::LidarConnection(std::string port, int baudrate)
{
	this->connect(port, baudrate);
}

void LidarConnection::read_versions()
{
	sl_lidar_response_device_info_t device_info;
	auto res = driver->getDeviceInfo(device_info);
	if (SL_IS_OK(res)) {
		std::stringstream str_buffer;

		str_buffer << unsigned(device_info.model);
		model_version = str_buffer.str();
		str_buffer.str("");

		str_buffer << (device_info.firmware_version >> 8) << "." << (device_info.firmware_version & 0xffu);
		firmware_version = str_buffer.str();
		str_buffer.str("");

		str_buffer << unsigned(device_info.hardware_version);
		hardware_version = str_buffer.str();
		str_buffer.str("");
	}
	else {
		if (res == SL_RESULT_OPERATION_TIMEOUT) {
			// you can check the detailed failure reason
			std::cerr << "error, operation time out." << std::endl;
			exit(-1);
		}
		else {
			// other unexpected result
			std::cerr << "error, unexpected error, code: " << res << std::endl;
			exit(-1);
		}
	}
}

void LidarConnection::read_health()
{
	sl_lidar_response_device_health_t health_info;
	auto res = driver->getHealth(health_info);
	if (SL_IS_OK(res)) {
		switch (health_info.status)
		{
		case SL_LIDAR_STATUS_OK:
			health_status = "OK";
			break;
		case SL_LIDAR_STATUS_WARNING:
			health_status = "WARNING";
			break;
		case SL_LIDAR_STATUS_ERROR:
			health_status = "ERROR";
			break;
		}
	}
	else {
		std::cerr << "error, cannot retrieve the lidar health code: " << res << std::endl;
		exit(-1);
	}
}

void LidarConnection::connect(std::string port, int baudrate)
{
	std::cout << "connecting to " << port << " with baudrate " << baudrate << std::endl;
	channel = *sl::createSerialPortChannel(port, baudrate);
	driver = *sl::createLidarDriver();

	// Checking for errors
	if (!driver) {
		std::cerr << "insufficent memory, exit" << std::endl;
		exit(-1);
	}

	if (SL_IS_FAIL((driver)->connect(channel))) {
		std::cerr << "error, cannot bind to the specified serial port " << port << std::endl;
		exit(-1);
	}

	if (!driver->isConnected()) {
		std::cerr << "failed to connect, exit" << std::endl;
		exit(-1);
	}

	read_versions();
	std::cout << "Model: " << model_version;
	std::cout << ", Firmware Version: " << firmware_version;
	std::cout << ", Hardware Version: " << hardware_version;

	read_health();
	std::cout << ", Health: " << health_status << std::endl;
}

void LidarConnection::destroy_connection()
{
	printf("destroying connection");
	delete channel;
	delete driver;
}
