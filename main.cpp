//
// Created by Riley Connors on 3/20/24.
//

#include "KalmanFilter.h"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "rapidcsv.h"
#include <cmath>
#include <fstream>
#include <ctime>
#include <algorithm>

int main() {
    int trackID;

    auto now = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(now);

    rapidcsv::Document doc("tracks.csv");

    std::cout << "Enter a track ID: ";
    std::cin >> trackID;

    std::vector<double> id, vx, vy, vz, az, el;

    id = doc.GetColumn<double>("track_id");
    vx = doc.GetColumn<double>("vel_x");
    vy = doc.GetColumn<double>("vel_y");
    vz = doc.GetColumn<double>("vel_z");
    az = doc.GetColumn<double>("az");
    el = doc.GetColumn<double>("el");

    int id120 = std::count(id.begin(), id.end(), trackID);


    Eigen::MatrixXd Data(id120, 6);
    int inc = 0;

    for (int i = 0; i < id.size(); i++) {
        if (id.at(i) == trackID) {
            Data(inc, 0) = id.at(i);
            Data(inc, 1) = vx.at(i);
            Data(inc, 2) = vy.at(i);
            Data(inc, 3) = vz.at(i);
            Data(inc, 4) = az.at(i);
            Data(inc, 5) = el.at(i);
            inc++;
        }
        else {
            continue;
        }
    }

    Eigen::MatrixXd H(5, 7);
    H << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0;

    Eigen::MatrixXd Q(7, 7);
    Q << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 1;

    Eigen::MatrixXd P(7,7);
    P << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 1;

    double velVar = pow(.91, 2);
    double azVar = pow(1, 2);
    double elVar = pow(3, 2);
    Eigen::MatrixXd R(5, 5);
    R << velVar, 0, 0, 0, 0,
         0, velVar, 0, 0, 0,
         0, 0, velVar, 0, 0,
         0, 0, 0, azVar, 0,
         0, 0, 0, 0, elVar;

    Eigen::VectorXd x0(7);
    x0 << Data(0, 1), Data(0, 2), Data(0, 3), Data(0, 4), Data(0, 5), 0, 0;

    double t0 = 0;
    double dt = .104;

    std::ofstream file("output.csv");
    file << "ID,vel_x,vel_y,vel_z,az,el,azdot,eldot\n";

    KalmanFilter kf(P, Q, H, R, dt);
    kf.init(x0, t0);
    Eigen::VectorXd z(5);
    for (int i = 0; i < Data.rows(); i++) {
        z << Data(i, 1), Data(i, 2), Data(i, 3), Data(i, 4), Data(i, 5);
        Eigen::VectorXd x_hat = kf.get_x_hat();
        file << trackID << "," << x_hat(0) << "," << x_hat(1) << "," << x_hat(2) << "," << x_hat(3) << "," << x_hat(4) << "," << x_hat(5) << "," << x_hat(6) << "\n";
        kf.predict();
        kf.update(z);
    }

    file.close();

    now = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(now);
    time_t elapsed_time = end_time - start_time;

    std::cout << "Time elapsed: " << elapsed_time << " seconds\n";



}












