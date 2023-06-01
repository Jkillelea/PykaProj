// Depends on libcsv3, libcsv-dev, libeigen3-dev
// Compile with g++ kalman.cpp -lcsv
#include <eigen3/Eigen/Eigen>
#include <csv.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstdlib>

std::string read_file(const std::string &name) {
    std::ifstream file;
    std::string buffer;
    std::string line;

    file.open(name);

    while(file){
        std::getline(file, line);
        buffer.append(line);
        buffer.append("\n");
    }

    file.close();

    return buffer;
}

std::vector<double> record_data;
std::vector<std::vector<double>> logfile_data;

// Called for each element if line
void field_callback(void *field, size_t field_len, void *user_data) {
    double field_data = std::strtod((char *) field, nullptr);
    record_data.push_back(field_data);
}

// Called when an entire record is found
void record_callback(int n, void *p) {
    logfile_data.push_back(record_data);
    record_data.clear();
}

int main(int argc, char *argv[]) {
    std::string data_log = read_file("log1.csv");

    csv_parser parser;
    if (csv_init(&parser, CSV_STRICT | CSV_APPEND_NULL) != 0) {
        // error handler
    }

    csv_parse(&parser, (void *)data_log.c_str(), data_log.length(), &field_callback, &record_callback, nullptr);
    csv_fini(&parser, field_callback, record_callback, nullptr);
    csv_free(&parser);


    /* Implment a kalman filter */

    // State transition matrix for unforced motion (Ax + Bu, u = 0)
    const Eigen::Matrix3d A {
     //  MSL dMSL AGL
        {1,  1,   0}, // MSL = MSL + dMSL
        {0,  1,   0}, // dMSL = dMSL
        {0,  1,   1}, // AGL = AGL + dMSL
    };

    // (Co)variance matrix
    Eigen::Matrix3d P {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
    };

    // (Co)variance growth per iteration, mostly guessing
    const Eigen::Matrix3d Q {
    //  MSL     dMSL      AGL
        {0.01,   1,        1   }, // MSL
        {1,      0.01,    10   }, // dMSL
        {1,      10,       0.01}, // AGL
    };

    // (Co)variance matrix of measurements, found experimentally
    const Eigen::Matrix4d R {
        {10.5038,  10.5059,   3.1679,   3.1304},
        {10.5059,  21.0118,   3.1664,   3.1409},
        {3.1679,   3.1664,    2.4990,   1.1770},
        {3.1304,   3.1409,    1.1770,  11.4385},
    };

    Eigen::MatrixXd H {
        //   MSL dMSL AGL
        {1,  0,   0},  // GPS     = MSL
        {0,  1,   0},  // dGPS    = dMSL
        {0,  0,   1},  // laser1  = AGL
        {0,  0,   1},  // laser2  = MSL - Terrain
    };


    double last_t = 0;
    double last_gps = 0;
    Eigen::Vector3d x {{0, 0, 0}}; // state vector

    for (auto &row : logfile_data) {
        double t      = row[0];
        double gps    = row[1];
        double laser1 = row[2];
        double laser2 = row[3];

        // Calculate derivatives
        double dt = t - last_t;
        double dgps = dt > 0 ? (gps - last_gps) / dt : 0;


        // Next predicted state and covariance
        auto x_next = A * x;
        auto P_next = A * P * A.transpose() + Q;

        // measurement vector
        Eigen::Vector4d z {{gps, dgps, laser1, laser2}};
        auto z_pred = H * x;

        // Calculate Kalman gain
        auto K = P_next * H.transpose() * ((H * P_next * H.transpose() + R).inverse());

        // Update
        x = x_next + K * (z - z_pred);
        P = P_next - K * H * P_next;

        std::cout << x << std::endl << std::endl;
    }

    return 0;
}
