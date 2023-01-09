

#include <iostream>
#include <fstream>
#include <sstream>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include "LidarConnection.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define SERIAL_PORT "COM5"
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
#define SERIAL_PORT "/dev/ttyUSB0"
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

struct MeasurementPoint {
    bool start_flag;
    float raw_angle;
    float distance;
    int quality;
};

void display_data(std::vector<MeasurementPoint>* output_data_point)
{
    for (int i = 0; i < output_data_point->size(); i++) {
        std::cout << "start_flag: " << output_data_point->at(i).start_flag << " ";
        std::cout << "raw_angle: " << output_data_point->at(i).raw_angle << " ";
        std::cout << "distance: " << output_data_point->at(i).distance << " ";
        std::cout << "quality: " << output_data_point->at(i).quality << std::endl;
    }
}

void write_to_csv(std::vector<MeasurementPoint>* output_data_point, std::string file_path)
{
    std::ofstream output_file;
    output_file.open(file_path);
    output_file << "start_flag,raw_angle,distance,quality\n";
    for (int i = 0; i < output_data_point->size(); i++) {
        output_file << output_data_point->at(i).start_flag << ",";
        output_file << output_data_point->at(i).raw_angle << ",";
        output_file << output_data_point->at(i).distance << ",";
        output_file << output_data_point->at(i).quality << "\n";
    }
    output_file.close();
}

float avg_dist_in_angle_range(std::vector<MeasurementPoint>* data_point) {
    float avg_dist = 0.0f;
    int count = 0;
    for (int i = 0; i < data_point->size(); i++)
    {
        float dist = data_point->at(i).distance;
        float angle = data_point->at(i).raw_angle;
        float epsilon = 20.0f;
        if ((dist > 0) && ((angle <= epsilon && angle >= 0) || (angle >= 360-epsilon && angle <= 360))) {
            count++;
            avg_dist += data_point->at(i).distance;
        }
        //if ((dist > 0) && ((angle <= 5 && angle >= 0) || (angle >= 355 && angle <= 360))) {
        //    count++;
        //    avg_dist += data_point->at(i).distance;
        //}
    }
    avg_dist = avg_dist / count;
    return avg_dist;
}

sl_result capture_data(sl::ILidarDriver* drv, std::vector<MeasurementPoint>* output_data_point)
{
    sl_result ans;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    
    //std::cout << "waiting for data..." << std::endl;

    ans = drv->grabScanDataHq(nodes, count, 0);
    if (SL_IS_OK(ans) || ans == SL_RESULT_OPERATION_TIMEOUT) {
        drv->ascendScanData(nodes, count);
        output_data_point->clear();
        for (int pos = 0; pos < (int)count; ++pos) {
            MeasurementPoint point;
            point.start_flag = (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT);
            point.raw_angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
            point.distance = nodes[pos].dist_mm_q2 / 4.0f;
            point.quality = nodes[pos].quality;
            output_data_point->push_back(point);
        }
    }
    else {
        printf("error code: %x\n", ans);
    }

    return ans;
}

int main(int argc, const char* argv[]) {
    // start connection
    LidarConnection* conn = new LidarConnection(SERIAL_PORT, 115200);
    sl::ILidarDriver* drv = conn->get_driver();

    // show all supported modes
    std::vector<sl::LidarScanMode> scanModes;
    drv->getAllSupportedScanModes(scanModes);
    std::cout << "[Supported modes]" << std::endl;
    for (int i = 0; i < scanModes.size(); i++) {
        std::cout << "id: " << scanModes[i].id << " ";
        std::cout << "us_per_sample: " << scanModes[i].us_per_sample << " ";
        std::cout << "max_distance: " << scanModes[i].max_distance << " ";
        std::cout << "ans_type: " << scanModes[i].ans_type << " ";
        std::cout << "scan_mode: " << scanModes[i].scan_mode << std::endl;
    }

    // start server
    
    // scan
    drv->setMotorSpeed();
    if (SL_IS_FAIL(drv->startScanExpress(0, 3))) {
        std::cerr << "error, cannot start the scan operation" << std::endl;
    }
    delay(3000);
    std::vector<MeasurementPoint> scan_data;
    while (true) {
        delay(300);
        scan_data.clear();
        if (SL_IS_FAIL(capture_data(drv, &scan_data)))
        {
            std::cerr << "error, cannot grab scan data" << std::endl;
        }
        //display_data(&scan_data);
        std::cout << "avg_dist: " << avg_dist_in_angle_range(&scan_data) << std::endl;
    }
    
    // stop connectionn
    conn->destroy_connection();
    delete conn;
    return 0;
}