#include "definitions.h"

#include "LidarConnection.h"

#include <iostream>
#include <sstream>
#include <cmath>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

// Constructor
lc::LidarConnection::LidarConnection(std::string port, int baudrate)
{
	this->connect(port, baudrate);
}

void lc::LidarConnection::read_versions()
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

void lc::LidarConnection::read_health()
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

void lc::LidarConnection::connect(std::string port, int baudrate)
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

void lc::LidarConnection::destroy_connection()
{
	printf("destroying connection");
	delete channel;
	delete driver;
}

sl_result lc::LidarConnection::capture_data(std::vector<lc::MeasurementPoint>* output_data_point)
{
	sl_result ans;

	sl_lidar_response_measurement_node_hq_t nodes[8192];
	size_t   count = _countof(nodes);

	//std::cout << "waiting for data..." << std::endl;
	//delay(300);

	ans = driver->grabScanDataHq(nodes, count, 20000U);

	if (SL_IS_OK(ans) || ans == SL_RESULT_OPERATION_TIMEOUT) {
		driver->ascendScanData(nodes, count);
		output_data_point->clear();
		for (int pos = 0; pos < (int)count; ++pos) {
			MeasurementPoint point;

			//Raw data
			point.start_flag = (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT);
			point.angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
			point.distance = nodes[pos].dist_mm_q2 / 4.0f;
			point.quality = nodes[pos].quality;

			point.x = point.distance * std::cosf((point.angle - 270.0f) * M_PI / 180.0f);
			point.y = point.distance * std::sinf((point.angle - 270.0f) * M_PI / 180.0f);

			//Correction (LINEAR REGRESSION)
			point.distance = 308.8217f + (0.7213f * point.distance);

			output_data_point->push_back(point);
		}
	}
	else {
		printf("error code: %x\n", ans);
	}
	return ans;
}
