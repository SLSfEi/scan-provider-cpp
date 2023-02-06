

#include <iostream>
#include <fstream>
#include <sstream>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include "LidarConnection.h"

#include "ActiveSocket.h"

#include "definitions.h"


void display_data(std::vector<lc::MeasurementPoint>* output_data_point)
{
    for (int i = 0; i < output_data_point->size(); i++) {
        std::cout << "start_flag: " << output_data_point->at(i).start_flag << " ";
        std::cout << "raw_angle: " << output_data_point->at(i).raw_angle << " ";
        std::cout << "distance: " << output_data_point->at(i).distance << " ";
        std::cout << "quality: " << output_data_point->at(i).quality << std::endl;
    }
}

std::string points_to_csv_string(std::vector<lc::MeasurementPoint>* output_data_point)
{
    std::stringstream output_stream;

    output_stream << "start_flag,raw_angle,distance,quality\n";
    for (int i = 0; i < output_data_point->size(); i++) {
        output_stream << output_data_point->at(i).start_flag << ",";
        output_stream << output_data_point->at(i).raw_angle << ",";
        output_stream << output_data_point->at(i).distance << ",";
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

int main(int argc, const char* argv[]) {
    // start connection
    lc::LidarConnection* conn = new lc::LidarConnection(SERIAL_PORT, 115200);
    sl::ILidarDriver* drv = conn->get_driver();

    drv->setMotorSpeed();
    if (SL_IS_FAIL(drv->startScanExpress(0, 3))) {
        std::cerr << "error, cannot start the scan operation" << std::endl;
    }
    delay(2000);

    // start client
    CActiveSocket* client;
    while (true) {
        // scan
        std::vector<lc::MeasurementPoint> scan_data;
        conn->capture_data(&scan_data);
        std::string scan_csv = "AABBCC" + points_to_csv_string(&scan_data) + "XXYYZZ";

        client = new CActiveSocket;
        client->Initialize();
        std::cout << "Main Loop" << std::endl;
        if (client->Open(SERVER_ADDR, SERVER_PORT)) {
            std::cout << "Sending" << std::endl;
            auto dataStr = scan_csv.c_str();
            int chunk_size = 1024;
            for (int i = 0; i <= strlen(dataStr); i += chunk_size) {
                // manage start and end index
                int start_ind = i;
                int end_ind = i + chunk_size - 1;
                if (end_ind > strlen(dataStr) - 1) {
                    end_ind = strlen(dataStr) - 1;
                }

                // cut string in range
                std::string chunk_data = "";
                for (int j = start_ind; j <= end_ind; j++) {
                    chunk_data += dataStr[j];
                }

                // sending logic
                int send_result = 0;
                do {
                    send_result = client->Send((const uint8*)chunk_data.c_str(), strlen(chunk_data.c_str()));
                    std::cout << std::endl << "sending chunk_size: " << strlen(chunk_data.c_str()) << std::endl;
                    std::cout << chunk_data;
                } while (!send_result);
                
            }
            std::cout << std::endl;

            std::cout << "Closing connection" << std::endl;
            client->Close();
            delete client;
        }
        break;
    }

    conn->destroy_connection();
    delete conn;
    return 0;
}