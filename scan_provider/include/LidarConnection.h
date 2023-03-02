#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

namespace lc {
	struct MeasurementPoint {
		bool start_flag;
		float angle;
		float distance;
		float x;
		float y;
		int quality;
	};

	class LidarConnection {
	private:
		sl::IChannel* channel = nullptr;
		sl::ILidarDriver* driver = nullptr;
		std::string model_version = "";
		std::string firmware_version = "";
		std::string hardware_version = "";
		std::string health_status = "";
	public:
		LidarConnection(std::string port, int baudrate);
		sl::IChannel* get_channel() { return channel; }
		sl::ILidarDriver* get_driver() { return driver; }
		void read_versions();
		void read_health();
		std::string get_model() { return model_version; }
		std::string get_firmware() { return firmware_version; }
		std::string get_hardware() { return hardware_version; }
		void connect(std::string port, int baudrate);
		void destroy_connection();
		sl_result LidarConnection::capture_data(std::vector<MeasurementPoint>* output_data_point);
	};

}