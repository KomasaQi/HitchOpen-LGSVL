#pragma once
#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

class MFASMC_Controller {
public:
    // 参数结构体
    struct Params {
        int Ly = 1;
        int Lu = 1;
        int Ny;
        int Nu;
        double lambda = 1.0;
        double mu = 1.0;
        std::vector<double> rho;
        double eta = 0.5;
        double epsilon = 1e-5;
        double uplim = 50.0;
        Eigen::MatrixXd phi_Lyp1;
        Eigen::MatrixXd u_cons;
        Eigen::MatrixXd du_cons;
        Eigen::MatrixXd Lambda;
        double Ts = 0.05;
        Eigen::MatrixXd K;
        Eigen::MatrixXd E;
        std::string mode = "mfasmc";
    };

    MFASMC_Controller(const Params& params);
    Eigen::VectorXd step(const Eigen::VectorXd& yk, const Eigen::VectorXd& ydes);

private:
    Params params_;
    Eigen::VectorXd u;
    Eigen::VectorXd y;
    Eigen::VectorXd dH;
    Eigen::MatrixXd Phi;
    Eigen::VectorXd ydes_k_1;
    Eigen::VectorXd ek_1;
    Eigen::VectorXd hk_1;
    Eigen::VectorXd fk_1;
    Eigen::MatrixXd A;




    void updatePhiEstimation(const Eigen::VectorXd& yk);
    Eigen::VectorXd calculateControlValue(const Eigen::VectorXd& yk, const Eigen::VectorXd& ydes);
    Eigen::VectorXd calculateControlValueSMC(const Eigen::VectorXd& yk, const Eigen::VectorXd& ydes);
    Eigen::VectorXd smcApproach(const Eigen::VectorXd& yk);
    void validateObservation(const Eigen::VectorXd& yk, const Eigen::VectorXd& ydes) const;
    void checkValidity();

    static Eigen::MatrixXd parseMatrix(const YAML::Node& node) {
        int rows = node.size();
        int cols = node[0].size();
        Eigen::MatrixXd mat(rows, cols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                mat(i,j) = node[i][j].as<double>();
            }
        }
        return mat;
    }

};
