//
// Created by Riley Connors on 3/20/24.
//

#include "KalmanFilter.h"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "rapidcsv.h"

int main() {
    rapidcsv::Document doc("tracks.csv");
    std::vector<double> id, vx, vy, vz, az, el;

    id = doc.GetColumn<double>("track_id");
    vx = doc.GetColumn<double>("vel_x");
    vy = doc.GetColumn<double>("vel_y");
    vz = doc.GetColumn<double>("vel_z");
    az = doc.GetColumn<double>("az");
    el = doc.GetColumn<double>("el");

    Eigen::MatrixXd F(id.size(), 6);
    int inc = 0;
    for (int i = 0; i < id.size(); i++) {
        if (id.at(i) == 102) {
            F(inc, 0) = id.at(i);
            F(inc, 1) = vx.at(i);
            F(inc, 2) = vy.at(i);
            F(inc, 3) = vz.at(i);
            F(inc, 4) = az.at(i);
            F(inc, 5) = el.at(i);
            inc++;
        }
        else {
            continue;
        }
    }

    Eigen::MatrixXd P = F.block(0, 0, inc, 6);

    Eigen::MatrixXd A(7, 7);
    A << 1, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 1;


}












