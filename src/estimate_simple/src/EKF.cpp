#include <EKF.hpp>

void EKF::reset(Eigen::Matrix<double, 3, 1> tmp) {
    Xe[0] = tmp[0];
    Xe[1] = tmp[1];
    Xe[2] = tmp[2];
    Xe[3] = Xe[4] = Xe[5] = 0;
    P = const_P.asDiagonal();
    Q = const_Q.asDiagonal();
    R = const_R.asDiagonal();
}

EKF::Vy EKF::predict(double dT) {
    Vy tmp;
    tmp[0] = Xe[0] + Xe[3] * dT;
    tmp[1] = Xe[1] + Xe[4] * dT;
    tmp[2] = Xe[2] + Xe[5] * dT;
    return tmp;
}

void EKF::update(const EKF::Vy &Y, const double dT) {
    Xp = Xe;
    Xp[0] += Xe[3] * dT;
    Xp[1] += Xe[4] * dT;
    Xp[2] += Xe[5] * dT;
    F = Mxx::Identity();
    F(0, 3) = F(1, 4) = F(2, 5) = dT;
    P = F * P * F.transpose() + Q;

    Yp[0] = Xp[0];
    Yp[1] = Xp[1];
    Yp[2] = Xp[2];
    H = Myx::Zero();
    H(0, 0) = H(1, 1) = H(2, 2) = 1;

    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    Xe = Xp + K * (Y - Yp);
    P = (Mxx::Identity() - K * H) * P;
}

EKF::Vx EKF::const_P;
EKF::Vx EKF::const_Q;
EKF::Vy EKF::const_R;