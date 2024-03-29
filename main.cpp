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
#include <filesystem>
#include <ctime>
#include <algorithm>

//sorts track ids by frequency
std::vector<int> sortIDs(const std::vector<double>& id);

int main() {
    int trackID;

    //read in data from csv
    rapidcsv::Document doc("tracks.csv");

    //get desired columns from csv
    std::vector<double> id, vx, vy, vz, az, el;
    id = doc.GetColumn<double>("track_id");
    vx = doc.GetColumn<double>("vel_x");
    vy = doc.GetColumn<double>("vel_y");
    vz = doc.GetColumn<double>("vel_z");
    az = doc.GetColumn<double>("az");
    el = doc.GetColumn<double>("el");

    //sort track ids by frequency
    std::vector<int> trackIDs = sortIDs(id);

    //loop through top 10 track ids
    for (int k = 0; k < 10; k++)
    {
        int uniqueid = trackIDs.at(k);
        int count = std::count(id.begin(), id.end(), uniqueid);

        //build data matrix for each track id
        Eigen::MatrixXd Data(count, 6);
        int inc = 0;
        for (int i = 0; i < id.size(); i++) {
            if (id.at(i) == uniqueid) {
                Data(inc, 0) = id.at(i);
                Data(inc, 1) = vx.at(i);
                Data(inc, 2) = vy.at(i);
                Data(inc, 3) = vz.at(i);
                Data(inc, 4) = az.at(i);
                Data(inc, 5) = el.at(i);
                inc++;
            } else {
                continue;
            }
        }

        //initialize matrices
        Eigen::MatrixXd H(5, 7);
        H << 1, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 1, 0, 0;
        Eigen::MatrixXd P(7, 7);
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
        double Qvar = 0;

        //loop through different Q values and output to csv
        for (int j = 0; j < 10; j++) {
            Eigen::MatrixXd Q(7, 7);
            Q << Qvar, 0, 0, 0, 0, 0, 0,
                    0, Qvar, 0, 0, 0, 0, 0,
                    0, 0, Qvar, 0, 0, 0, 0,
                    0, 0, 0, Qvar, 0, 0, 0,
                    0, 0, 0, 0, Qvar, 0, 0,
                    0, 0, 0, 0, 0, Qvar, 0,
                    0, 0, 0, 0, 0, 0, Qvar;

            //create directory for each track id
            std::filesystem::path dir = std::to_string(uniqueid);
            if (!std::filesystem::exists(dir)) {
                std::filesystem::create_directory(dir);
            }

            //create output file and header
            std::string filename = "output" + std::to_string(j) + ".csv";
            std::ofstream file(dir / filename);
            file << "ID,vel_x,vel_y,vel_z,az,el,azdot,eldot\n";

            //initialize Kalman filter
            KalmanFilter kf(P, Q, H, R, dt);
            kf.init(x0, t0);
            Eigen::VectorXd z(5);

            //loop through data, apply filter and write to file
            for (int i = 0; i < Data.rows(); i++) {
                z << Data(i, 1), Data(i, 2), Data(i, 3), Data(i, 4), Data(i, 5);
                Eigen::VectorXd x_hat = kf.get_x_hat();
                file << uniqueid << "," << x_hat(0) << "," << x_hat(1) << "," << x_hat(2) << "," << x_hat(3) << ","
                     << x_hat(4) << "," << x_hat(5) << "," << x_hat(6) << "\n";
                kf.predict();
                kf.update(z);
            }
            file.close();
            Qvar += .1;
        }
        std::cout << "Done with track ID " << uniqueid << std::endl;
    }
}

std::vector<int> sortIDs(const std::vector<double>& id) {
    std::map<int, int> idCount;
    for (double i : id) {
        int stati = static_cast<int>(i);
        if (idCount.find(stati) == idCount.end()) {
            idCount[stati] = 1;
        }
        else {
            idCount[stati]++;
        }
    }
    std::vector<std::pair<int, int>> idCountVec = std::vector<std::pair<int, int>>(idCount.begin(), idCount.end());
    std::sort(idCountVec.begin(), idCountVec.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return a.second > b.second;
    });
    std::vector<int> idVec;
    for (auto & i : idCountVec) {
        idVec.push_back(i.first);
    }
    return idVec;
}











