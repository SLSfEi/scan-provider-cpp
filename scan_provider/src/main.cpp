#include <iostream>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <cmath>
#include <random>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <cpr/cpr.h>

#include "LidarConnection.h"
#include "definitions.h"

#include "INIReader.h"


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

void check_config_var(std::string name, std::string var, std::string default="UNDEFINED"){
    if(var != default){
        std::cout << name << ": " << var << std::endl;
    } else {
        std::cout << name << " is undefined, exiting" << std::endl;
        exit(1);
    }
}

bool check_config_var(std::string name, int var, int default=-1, bool will_exit=true){
    if(var != default){
        std::cout << name << ": " << var << std::endl;
        return true;
    } else {
        std::cout << name << " is undefined, exiting" << std::endl;
        if(will_exit){
            exit(1);
        }
        return false;
    }
}

void gen_random_scan_data(std::vector<lc::MeasurementPoint>* output_data_point){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0,3);
    float r = dis(gen);
    float cur_angle = 0;
    float distance = 6000;
    float quality = 0;
    output_data_point->clear();
    while(cur_angle + r < 360){
        cur_angle += r;
        r = r = dis(gen);

        lc::MeasurementPoint point;
		point.start_flag = false;
		point.angle = cur_angle;
		point.distance = distance;
		point.quality = quality;

		point.x = point.distance * std::cosf((point.angle - 270.0f) * M_PI / 180.0f);
		point.y = point.distance * std::sinf((point.angle - 270.0f) * M_PI / 180.0f);

		output_data_point->push_back(point);
    }
}


int main(int argc, const char* argv[]) {

    // dealing with exit (destroy driver first)
    signal(SIGINT, ctrlc);
    signal(SIGABRT, ctrlc);
    signal(SIGTERM, ctrlc);

    // load config file
    std::cout << "==========[" << "config" << "]==========" << std::endl;
    INIReader reader("./config.ini");
    if (reader.ParseError() != 0) {
        std::cout << "error, can't load 'config.ini'\n";
        return 1;
    }

    auto server_endpoint = reader.Get("connection","server_endpoint","UNDEFINED");
    auto serial_port = reader.Get("serial","port","UNDEFINED");
    auto serial_baudrate = reader.GetInteger("serial","baudrate",-1);
    auto base_delay = reader.GetInteger("retry","base_delay", -1);
    auto max_delay = reader.GetInteger("retry","max_delay", -1);
    auto gen_random = reader.GetBoolean("debug","gen_random",false);
    std::cout << "config.ini loaded" << std::endl;
    check_config_var("server_endpoint", server_endpoint);
    check_config_var("serial_port", serial_port);
    check_config_var("serial_baudrate", serial_baudrate);
    check_config_var("base_delay", base_delay);
    check_config_var("max_delay", max_delay);
    if(gen_random){
        std::cout << "gen_random: true" << std::endl;
    }
    std::cout << "all config loaded" << std::endl;
    std::cout << std::endl;

    if(gen_random){
        std::cout << "[DEBUG] Generating random data" << std::endl;
        std::cout << "==========[" << "scan" << "]==========" << std::endl;

        std::vector<lc::MeasurementPoint> scan_data;
        std::string scan_csv;

        while(true){
        delay(150);
        // send data here
        gen_random_scan_data(&scan_data);
        scan_csv = points_to_csv_string(&scan_data);
        cpr::Response r = cpr::Post(cpr::Url{ server_endpoint },
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
    }

    std::cout << "==========[" << "lidar" << "]==========" << std::endl;
    // start connection
    lc::LidarConnection* conn = new lc::LidarConnection(serial_port, serial_baudrate);
    sl::ILidarDriver* drv = conn->get_driver();
    std::cout << std::endl;

    std::cout << "==========[" << "scan" << "]==========" << std::endl;
    drv->setMotorSpeed();
    if (SL_IS_FAIL(drv->startScanExpress(0, 3))) {
        std::cerr << "error, cannot start the scan operation" << std::endl;
    }
    delay(2000);

    bool is_failing = false;
    int cur_delay = 300;
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
                } else {
                    cur_delay = max_delay;
                }
            } else
            {
                is_failing = false;
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
        cpr::Response r = cpr::Post(cpr::Url{ server_endpoint },
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