#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <iostream>
#include <string>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {

public:
    KalmanFilter();

private:
    //tracks time step
    unsigned int k;

    // m - input dimension, n - state dimension, r - output dimension
    unsigned int m, n, r;

    // ABC are system matrices, QR are noise covariance matrices, P0 is initial state covariance
    MatrixXd A, B, C, Q, R, P0;
    // x0 is initial state
    MatrixXd x0;

    MatrixXd estimatesAposteriori, estimatesApriori;

    MatrixXd covarianceAposteriori, covarianceApriori;

    MatrixXd kalmanGain;

    MatrixXd errors;
};

#endif //KALMANFILTER_H

