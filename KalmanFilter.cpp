//
// Created by Riley Connors on 3/14/24.
//
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() = default;


KalmanFilter::KalmanFilter(Eigen::MatrixXd &F, Eigen::MatrixXd &P, Eigen::MatrixXd &Q, Eigen::MatrixXd &H, Eigen::MatrixXd &R, Eigen::MatrixXd &K, double &dt)
: F(F), P(P), Q(Q), H(H), R(R), K(K), dt(dt), m(this->H.rows()), n(this->H.cols()), x_hat(n), x_hat_new(n), w(n), e(m), v(m), I(n, n) {
    I.setIdentity();
}

void KalmanFilter::init(Eigen::VectorXd &x0, double &t0) {
    this-> initialized = true;
    this->x_hat = x0;
    this->t0 = t0;
    t = t0;
}

void KalmanFilter::predict() {
    if (!initialized) {
        std::cerr << "KalmanFilter::predict() - Not initialized!" << std::endl;
        return;
    }
    x_hat = F * x_hat + w;
    P = F * P * F.transpose() + Q;

    t += dt;
}

void KalmanFilter::update(Eigen::VectorXd &z) {
    if (!initialized ) {
        std::cerr << "KalmanFilter::update() - Not initialized!" << std::endl;
        return;
    }
    e = z - (H * x_hat + v);
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x_hat = x_hat + K * e;
    P = (I - K * H) * P;
}

double KalmanFilter::f4_vx(Eigen::VectorXd &x, double &t) {
    return (t * x(2)) / (pow(x(0), 2) + pow(x(2), 2));
}






