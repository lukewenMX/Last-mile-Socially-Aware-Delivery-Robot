#include <Eigen/Eigen>

class EKF {
   public:
    using Mxx = Eigen::Matrix<double, 6, 6>;
    using Myx = Eigen::Matrix<double, 3, 6>;
    using Mxy = Eigen::Matrix<double, 6, 3>;
    using Myy = Eigen::Matrix<double, 3, 3>;
    using Vx = Eigen::Matrix<double, 6, 1>;
    using Vy = Eigen::Matrix<double, 3, 1>;

    // Xp = f(last_Xe) + Q
    // Xe = h(Xp) + R
    static Vx const_P, const_Q;
    static Vy const_R;

    Vx Xe;  // Xe 估计状态变量(滤波后的状态)
    Vx Xp;  // Xp 预测状态变量
    Mxx F;  // F 预测雅克比
    Myx H;  // H 观测雅克比
    Mxx P;  // P 状态协方差
    Mxx Q;  // Q 预测过程协方差
    Myy R;  // R 观测过程协方差
    Mxy K;  // K 卡尔曼增益
    Vy Yp;  // Yp 预测观测量

    explicit EKF(const Vx &X0 = Vx::Zero())
        : Xe(X0), P(Mxx::Identity()), Q(Mxx::Identity()), R(Myy::Identity()) {}

    void reset(Eigen::Matrix<double, 3, 1> tmp);

    //该函数只是单纯地计算 dT 秒之后的坐标
    Vy predict(double dT);

    void update(const Vy &Y, const double dT);
    static void init(const Vx& _P, const Vx& _Q, const Vy& _R){
        const_P = _P;
        const_Q = _Q;
        const_R = _R;
    }
    // static void init(const toml::value &config) {
    //     toml_to_matrix(config.at("P"), const_P);
    //     toml_to_matrix(config.at("Q"), const_Q);
    //     toml_to_matrix(config.at("R"), const_R);
    //     toml_to_matrix(config.at("Ke"), const_Ke);
    // }
};