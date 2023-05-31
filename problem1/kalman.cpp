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


    Eigen::MatrixXd A {
        // AGL dAGL Terrain dTerrain
        {1,    1,   0,     -1}, // AGL
        {0,    1,   0,      0}, // dAGL
        {0,    0,   1,      1}, // Terrain
        {0,   -1,   0,      1}, // dTerrain
    };

    // Eigen::MatrixXd P = TODO
    // Eigen::MatrixXd Q = TODO

    // Covariance matrix of measurements
    Eigen::MatrixXd R {
        { 5.6191e-01,  3.7912e-04, -1.4529e-02,  1.8362e-04, -8.5454e-03,  1.4775e-04},
        { 3.7912e-04,  2.6099e-05, -7.2327e-05,  2.4852e-06, -8.1877e-05,  3.0357e-06},
        {-1.4529e-02, -7.2327e-05,  2.7869e-02,  1.2229e-04,  2.7874e-02,  1.5943e-04},
        { 1.8362e-04,  2.4852e-06,  1.2229e-04,  2.8554e-04, -1.3987e-04,  5.9946e-05},
        {-8.5454e-03, -8.1877e-05,  2.7874e-02, -1.3987e-04,  2.9542e-02,  1.2854e-04},
        { 1.4775e-04,  3.0357e-06,  1.5943e-04,  5.9946e-05,  1.2854e-04,  2.9498e-04},
    };

    Eigen::MatrixXd H {
        // AGL dAGL Terrain dTerrain
        {1,    0,   1,      0}, // GPS     = AGL + Terrain
        {0,    1,   0,     -1}, // dGPS    = dAGL - dTerrain
        {1,    0,   0,      0}, // laser1  = AGL
        {0,    1,   0,     -1}, // dlaser1 = dAGL - dTerrain
        {1,    0,   0,      0}, // laser2  = AGL
        {0,    1,   0,     -1}, // dlaser2 = dAGL - dTerrain
    };


    double last_t = 0;
    double last_gps = 0;
    double last_laser1 = 0;
    double last_laser2 = 0;
    Eigen::VectorXd state {{0, 0, 0, 0}};
    Eigen::VectorXd last_z {{0, 0, 0, 0, 0, 0}};

    for (auto &row : logfile_data) {
        double t      = row[0];
        double gps    = row[1];
        double laser1 = row[2];
        double laser2 = row[3];

        // Calculate derivatives
        double dt = t - last_t;
        double dgps = dt > 0 ? (gps - last_gps) / dt : 0;
        double dlaser1 = dt > 0 ? (laser1 - last_laser1) / dt : 0;
        double dlaser2 = dt > 0 ? (laser2 - last_laser2) / dt : 0;

        Eigen::VectorXd z {{gps, dgps, laser1, dlaser1, laser2, dlaser2}};

        auto state_next = A * state;

        std::cout << state_next << std::endl << std::endl;
    }

    return 0;
}
