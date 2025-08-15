#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include <string>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

// Type aliases for clarity
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix8x6d = Eigen::Matrix<double, 6, 8>;

class VehicleModel {
public:
    VehicleModel(const std::string& config_path);
    std::pair<Vector6d, Vector6d> dynamics(const Vector6d& eta, const Vector6d& nu, const Eigen::Vector<double, 8>& tau_p) const;

private:
    void load_config(const std::string& path);
    void build_matrices();
    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& a) const;
    Matrix6d transformation_matrix(const Vector6d& eta) const;
    Matrix6d coriolis_matrix(const Vector6d& nu) const;
    Matrix6d damping_matrix(const Vector6d& nu) const;
    Vector6d restoring_forces(const Vector6d& eta) const;

    // --- Member Variables ---
    nlohmann::json config_json_; // Store the loaded JSON data
    double mass_, g_, W_, B_, W_minus_B_;
    Eigen::Vector3d r_g_, r_B_;
    Eigen::Matrix3d I_cg_;
    Eigen::Matrix3d skew_m_;
    Eigen::Matrix3d skew_I_;
    Matrix6d D_l_, D_n_;
    Matrix8x6d A_thruster_alloc_;
    Matrix6d M_inv_;
    
    // Intermediate matrices for Coriolis calculation
    Eigen::Matrix3d A11_added_mass_, A22_added_mass_;
};

#endif // VEHICLE_MODEL_H