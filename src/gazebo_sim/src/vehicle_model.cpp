#include "gazebo_sim/vehicle_model.h"
#include <iostream>
#include <fstream>
#include <cmath>

using json = nlohmann::json;

VehicleModel::VehicleModel(const std::string& config_path) {
    load_config(config_path);
    build_matrices();
    std::cout << "VehicleModel initialized successfully from " << config_path << std::endl;
}

void VehicleModel::load_config(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open config file: " + path);
    }
    this->config_json_ = json::parse(f);
    json config = this->config_json_["assembly_mass_properties"];

    mass_ = config["mass"]["value"].get<double>();
    g_ = config["dynamics"]["g"]["value"].get<double>();
    double fluid_density = config["dynamics"]["fluid_density"]["value"].get<double>();
    double displaced_volume = config["dynamics"]["displaced_volume"]["value"].get<double>();
    W_ = mass_ * g_;
    B_ = fluid_density * displaced_volume * g_;
    W_minus_B_ = W_ - B_;

    r_g_ << config["center_of_mass"]["X"].get<double>(),
            config["center_of_mass"]["Y"].get<double>(),
            config["center_of_mass"]["Z"].get<double>();
            
    r_B_ << config["center_of_buoancy"]["X"].get<double>(),
            config["center_of_buoancy"]["Y"].get<double>(),
            config["center_of_buoancy"]["Z"].get<double>();

    auto moments_com = config["moments_of_inertia_about_center_of_mass"];
    I_cg_ << moments_com["Lxx"].get<double>(), -moments_com["Lxy"].get<double>(), -moments_com["Lxz"].get<double>(),
            -moments_com["Lyx"].get<double>(),  moments_com["Lyy"].get<double>(), -moments_com["Lyz"].get<double>(),
            -moments_com["Lzx"].get<double>(), -moments_com["Lzy"].get<double>(),  moments_com["Lzz"].get<double>();

    auto damping = config["dynamics"]["damping"];
    D_l_ = Matrix6d::Zero();
    D_l_(0, 0) = damping["linear"]["D_u"].get<double>();
    D_l_(1, 1) = damping["linear"]["D_v"].get<double>();
    D_l_(2, 2) = damping["linear"]["D_w"].get<double>();
    D_l_(3, 3) = damping["angular"]["D_p"].get<double>();
    D_l_(4, 4) = damping["angular"]["D_q"].get<double>();
    D_l_(5, 5) = damping["angular"]["D_r"].get<double>();
    
    D_n_ = Matrix6d::Zero();
    D_n_(0, 0) = damping["linear_n"]["Dn_u"].get<double>();
    D_n_(1, 1) = damping["linear_n"]["Dn_v"].get<double>();
    D_n_(2, 2) = damping["linear_n"]["Dn_w"].get<double>();
    D_n_(3, 3) = damping["angular_n"]["Dn_p"].get<double>();
    D_n_(4, 4) = damping["angular_n"]["Dn_q"].get<double>();
    D_n_(5, 5) = damping["angular_n"]["Dn_r"].get<double>();

    auto dim = config["dimensions"];
    double lm = dim["lm"].get<double>();
    double wf = dim["wf"].get<double>();
    double lr = dim["lr"].get<double>();
    double rf = dim["rf"].get<double>();
    double a_rad = config["rear_propeller_angle"]["value"].get<double>() * M_PI / 180.0;
    
    A_thruster_alloc_ << 1, 1, cos(a_rad),  cos(a_rad), 0,    0,    0,    0,
                         0, 0, sin(a_rad),  sin(a_rad), 0,    0,    0,    0,
                         0, 0, 0,           0,          1,    1,    1,    1,
                         0, 0, 0,           0,          lm,  -lm,   lm,  -lm,
                         0, 0, 0,           0,          rf,   rf,  -rf,  -rf,
                         -wf, wf, -lr*sin(a_rad), lr*sin(a_rad), 0, 0, 0, 0;
}

void VehicleModel::build_matrices() {
    Matrix6d M_rb = Matrix6d::Zero();
    M_rb.block<3, 3>(0, 0) = mass_ * Eigen::Matrix3d::Identity();
    M_rb.block<3, 3>(0, 3) = -mass_ * skew_symmetric(r_g_);
    M_rb.block<3, 3>(3, 0) =  mass_ * skew_symmetric(r_g_);
    M_rb.block<3, 3>(3, 3) = I_cg_;

    json C = this->config_json_["assembly_mass_properties"]["added_mass"];
    
    Eigen::Matrix3d A11 = Eigen::Matrix3d::Zero();
    A11(0,0) = -C["C_X"].get<double>();
    A11(1,1) = -C["C_Y"].get<double>();
    A11(2,2) = -C["C_Z"].get<double>();
    
    Eigen::Matrix3d A12 = Eigen::Matrix3d::Zero();
    A12(1,2) = -C["C_Z_q"].get<double>();
    A12(2,1) = -C["C_Y_r"].get<double>();

    Eigen::Matrix3d A21 = A12.transpose();
    
    Eigen::Matrix3d A22 = Eigen::Matrix3d::Zero();
    A22(0,0) = -C["C_K"].get<double>();
    A22(1,1) = -C["C_M"].get<double>();
    A22(2,2) = -C["C_N"].get<double>();
    
    Matrix6d M_a = Matrix6d::Zero();
    M_a.block<3,3>(0,0) = A11;
    M_a.block<3,3>(0,3) = A12;
    M_a.block<3,3>(3,0) = A21;
    M_a.block<3,3>(3,3) = A22;

    Matrix6d M_total = M_rb + M_a;
    M_inv_ = M_total.inverse();

    // Store these for the Coriolis calculation
    A11_added_mass_ = A11;
    A22_added_mass_ = A22;
}

std::pair<Vector6d, Vector6d> VehicleModel::dynamics(const Vector6d& eta, const Vector6d& nu, const Eigen::Vector<double, 8>& tau_p) const {
    Vector6d tau = A_thruster_alloc_ * tau_p;
    Matrix6d J = transformation_matrix(eta);
    Matrix6d C = coriolis_matrix(nu);
    Matrix6d D = damping_matrix(nu);
    Vector6d g = restoring_forces(eta);
    Vector6d eta_dot = J * nu;
    Vector6d nu_dot = M_inv_ * (tau - C * nu - D * nu - g);
    return {eta_dot, nu_dot};
}

Eigen::Matrix3d VehicleModel::skew_symmetric(const Eigen::Vector3d& a) const {
    Eigen::Matrix3d S;
    S <<  0,    -a(2),  a(1),
         a(2),    0,   -a(0),
        -a(1),  a(0),    0;
    return S;
}

Matrix6d VehicleModel::transformation_matrix(const Vector6d& eta) const {
    double phi = eta(3), theta = eta(4), psi = eta(5);
    double cphi = cos(phi), sphi = sin(phi);
    double cth = cos(theta), sth = sin(theta);
    double cpsi = cos(psi), spsi = sin(psi);
    
    Eigen::Matrix3d R;
    R << cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi,
         spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi,
         -sth,     cth*sphi,                  cth*cphi;

    Eigen::Matrix3d T;
    if (std::abs(cth) < 1e-6) { T = Eigen::Matrix3d::Identity(); } 
    else { T << 1, sphi*tan(theta), cphi*tan(theta), 0, cphi, -sphi, 0, sphi/cth, cphi/cth; }

    Matrix6d J = Matrix6d::Zero();
    J.block<3, 3>(0, 0) = R;
    J.block<3, 3>(3, 3) = T;
    return J;
}

// ==============================================================================
//           DEFINITIVE, STANDARD FOSSEN MODEL CORIOLIS MATRIX
// ==============================================================================
Matrix6d VehicleModel::coriolis_matrix(const Vector6d& nu) const {
    Eigen::Vector3d nu1 = nu.head<3>();
    Eigen::Vector3d nu2 = nu.tail<3>();

    // --- C_rb (Rigid Body) Term ---
    Matrix6d C_rb = Matrix6d::Zero();
    C_rb.block<3, 3>(3, 3) = -skew_symmetric(I_cg_ * nu2);

    // --- C_a (Added Mass) Term ---
    Matrix6d C_a = Matrix6d::Zero();
    C_a.block<3, 3>(0, 3) = -skew_symmetric(A11_added_mass_ * nu1);
    C_a.block<3, 3>(3, 0) = -skew_symmetric(A11_added_mass_ * nu1);
    C_a.block<3, 3>(3, 3) = -skew_symmetric(A22_added_mass_ * nu2);

    return C_rb + C_a;
}

Matrix6d VehicleModel::damping_matrix(const Vector6d& nu) const {
    Matrix6d D_quadratic = D_n_;
    for (int i = 0; i < 6; ++i) { D_quadratic(i, i) *= std::abs(nu(i)); }
    return D_l_ + D_quadratic;
}

Vector6d VehicleModel::restoring_forces(const Vector6d& eta) const {
    double phi = eta(3), theta = eta(4);
    double cth = cos(theta), sth = sin(theta), sphi = sin(phi), cphi = cos(phi);
    
    Vector6d g_eta;
    g_eta << (W_ - B_) * sth,
             -(W_ - B_) * cth * sphi,
             -(W_ - B_) * cth * cphi,
             -(r_g_(1) * W_ - r_B_(1) * B_) * cth * cphi + (r_g_(2) * W_ - r_B_(2) * B_) * cth * sphi,
              (r_g_(2) * W_ - r_B_(2) * B_) * sth + (r_g_(0) * W_ - r_B_(0) * B_) * cth * cphi,
             -(r_g_(0) * W_ - r_B_(0) * B_) * cth * sphi - (r_g_(1) * W_ - r_B_(1) * B_) * sth;
    return g_eta;
}