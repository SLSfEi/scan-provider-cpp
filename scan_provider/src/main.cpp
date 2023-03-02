

#include <iostream>
#include <fstream>
#include <sstream>

#include <signal.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include "LidarConnection.h"

#include <cpr/cpr.h>

#include "definitions.h"


void display_data(std::vector<lc::MeasurementPoint>* output_data_point)
{
    for (int i = 0; i < output_data_point->size(); i++) {
        std::cout << "start_flag: " << output_data_point->at(i).start_flag << " ";
        std::cout << "angle: " << output_data_point->at(i).angle << " ";
        std::cout << "distance: " << output_data_point->at(i).distance << " ";
        std::cout << "x: " << output_data_point->at(i).x << " ";
        std::cout << "y: " << output_data_point->at(i).y << " ";
        std::cout << "quality: " << output_data_point->at(i).quality << std::endl;
    }
}

std::string points_to_csv_string(std::vector<lc::MeasurementPoint>* output_data_point)
{
    std::stringstream output_stream;

    output_stream << "start_flag,angle,distance,x,y,quality\n";
    for (int i = 0; i < output_data_point->size(); i++) {
        output_stream << output_data_point->at(i).start_flag << ",";
        output_stream << output_data_point->at(i).angle << ",";
        output_stream << output_data_point->at(i).distance << ",";
        output_stream << output_data_point->at(i).x << ",";
        output_stream << output_data_point->at(i).y << ",";
        output_stream << output_data_point->at(i).quality << "\n";
    }
    return output_stream.str();
}

float avg_dist_in_angle_range(std::vector<lc::MeasurementPoint>* data_point) {
    float avg_dist = 0.0f;
    int count = 0;
    for (int i = 0; i < data_point->size(); i++)
    {
        float dist = data_point->at(i).distance;
        float angle = data_point->at(i).angle;
        float epsilon = 20.0f;
        if ((dist > 0) && ((angle <= epsilon && angle >= 0) || (angle >= 360-epsilon && angle <= 360))) {
            count++;
            avg_dist += data_point->at(i).distance;
        }
    }
    avg_dist = avg_dist / count;
    return avg_dist;
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char* argv[]) {

    signal(SIGINT, ctrlc);
    signal(SIGABRT, ctrlc);
    signal(SIGTERM, ctrlc);

    // start connection
    lc::LidarConnection* conn = new lc::LidarConnection(SERIAL_PORT, 115200);
    sl::ILidarDriver* drv = conn->get_driver();

    drv->setMotorSpeed();
    if (SL_IS_FAIL(drv->startScanExpress(0, 3))) {
        std::cerr << "error, cannot start the scan operation" << std::endl;
    }
    delay(2000);


    bool is_failing = FALSE;
    int base_delay = 300;
    int cur_delay = 300;
    int max_delay = 3000;
    
    while (true) {

        if (ctrl_c_pressed) {
            break;
        }

        // scan
        std::vector<lc::MeasurementPoint> scan_data;

        // delay(200);
        if (SL_IS_FAIL(conn->capture_data(&scan_data))) {
            if (is_failing) {
                std::cout << "lidar failing" << std::endl;
                if(cur_delay + cur_delay <= max_delay){
                    cur_delay += cur_delay;
                }
            } else
            {
                is_failing = TRUE;
            }
            delay(cur_delay);
            continue;
        }
        else
        {
            is_failing = FALSE;
        }
        std::string scan_csv = points_to_csv_string(&scan_data);

        // send data here
        cpr::Response r = cpr::Post(cpr::Url{ SERVER_ENDPOINT },
            cpr::Body{ scan_csv },
            cpr::Header{ {"Content-Type", "text/csv"} });
        std::cout << "[HTTP-" << r.status_code << "] ";
        if (r.status_code == 0) {
            std::cout << "cannot connect to server" << std::endl;
            continue;
        }
        if (r.text == "{\"status\": \"OK\"}") {
            std::cout << "valid response" << std::endl;
            continue;
        }
        else {
            std::cout << "invalid response" << std::endl;
            continue;
        }
        std::cout << std::endl;

    }
    std::cout << "exiting program" << std::endl;
    conn->destroy_connection();
    delete conn;
    return 0;
}