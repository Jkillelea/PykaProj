// Depends on libcsv3, libcsv-dev, libeigen3-dev
// Compile with g++ kalman.cpp -lcsv
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <csv.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstdlib>

#define CHECK_NAN(x) \
do { \
    if (x.hasNaN()) { \
            std::cout << #x << " has NaN!" << std::endl; \
            std::cout << "Iteration: " << t << std::endl; \
            std::cout << x << std::endl; \
    } \
} while (0)


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

// Called for each element of line
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
        {1,  1,   0}, //  MSL = MSL + dMSL
        {0,  1,   0}, // dMSL = dMSL
        {0,  1,   1}, //  AGL = AGL + dMSL
    };

    // (Co)variance matrix
    Eigen::Matrix3d P {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
    };

    // (Co)variance growth per iteration, mostly guessing
    const Eigen::Matrix3d Q = 0.01 * Eigen::Matrix3d::Identity();

    // (Co)variance matrix of measurements, found experimentally
    const Eigen::Matrix4d R {
        {10.5038,  10.5059,   3.1679,   3.1304},
        {10.5059,  21.0118,   3.1664,   3.1409},
        {3.1679,   3.1664,    2.4990,   1.1770},
        {3.1304,   3.1409,    1.1770,  11.4385},
    };

    // State to measurement matrix.
    Eigen::MatrixXd H {
      // MSL dMSL AGL
        {1,  0,   0},  //  GPS    = MSL
        {0,  1,   0},  // dGPS    = dMSL
        {0,  0,   1},  //  laser1 = AGL
        {0,  0,   1},  //  laser2 = AGL
    };


    double last_t = 0;
    double last_gps = 0;
    Eigen::Vector3d x {{0, 0, 0}}; // state vector

    for (auto &row : logfile_data) {
        double t      = row[0];
        double gps    = row[1];
        double laser1 = row[2];
        double laser2 = row[3];

        // Calculate derivative of GPS altitude to get aircraft vertical velocity
        double dt = t - last_t;
        double dgps = dt > 0 ? (gps - last_gps) / dt : 0;


        // Next predicted state and covariance
        auto x_next = A * x;
        CHECK_NAN(x_next);

        auto At = A.transpose();
        auto P_next = A * P * At + Q;
        CHECK_NAN(P_next);

        // measurement vector
        Eigen::Vector4d z {{gps, dgps, laser1, laser2}};

        // Calculate Kalman gain
        auto Ht = H.transpose();
        Eigen::MatrixXd K = P_next * Ht * (H * P_next * Ht + R).inverse();
        CHECK_NAN(K);

        // Update
        Eigen::Vector4d z_pred = H * x;
        CHECK_NAN(z_pred);

        x = x_next + K * (z - z_pred);
        CHECK_NAN(x);

        P = P_next - K * (H * P_next);
        CHECK_NAN(P);

        // Output timestamp and AGL altitude
        std::cout << t << "," << x[2] << std::endl;
    }

    return 0;
}
