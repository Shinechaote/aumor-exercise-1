#include "kalman_positioning/ukf.hpp"
#include <iostream>
#include <map>
#include <Eigen/Geometry>

/**
 * STUDENT ASSIGNMENT: Unscented Kalman Filter Implementation
 * 
 * This file contains placeholder implementations for the UKF class methods.
 * Students should implement each method according to the UKF algorithm.
 * 
 * Reference: Wan, E. A., & Van Der Merwe, R. (2000). 
 * "The Unscented Kalman Filter for Nonlinear Estimation"
 */

// ============================================================================
// CONSTRUCTOR
// ============================================================================

/**
 * @brief Initialize the Unscented Kalman Filter
 * 
 * STUDENT TODO:
 * 1. Initialize filter parameters (alpha, beta, kappa, lambda)
 * 2. Initialize state vector x_ with zeros
 * 3. Initialize state covariance matrix P_ 
 * 4. Set process noise covariance Q_
 * 5. Set measurement noise covariance R_
 * 6. Calculate sigma point weights for mean and covariance
 */
UKF::UKF(double process_noise_xy, double process_noise_theta,
         double measurement_noise_xy, int num_landmarks)
    : nx_(5), nz_(2) {
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    std::cout << "UKF Constructor: TODO - Implement filter initialization" << std::endl;
    
    x_.resize(nx_);
    x_ << 0, 0, 0, 0, 0;
    
    P_.resize(nx_,nx_);
    P_.setIdentity();

    Q_.resize(nx_,nx_);
    Q_.setZero();
    Q_(0,0) = process_noise_xy;
    Q_(1,1) = process_noise_xy;
    Q_(2,2) = process_noise_theta;

    R_.resize(nz_,nz_);
    R_.setZero();
    R_(0,0) = measurement_noise_xy;
    R_(1,1) = measurement_noise_xy;

    // Parameters
    lambda_ = pow(ALPHA, 2) * (nx_ + KAPPA) - nx_;
    gamma_ = std::sqrt(nx_ + lambda_);

    // Weights
    Wm_.clear();
    Wm_.push_back(lambda_ / (nx_ + lambda_));
    for (int i = 1; i < 2 * nx_ + 1; i++){
        Wm_.push_back(1.0 / (2.0 * (nx_ + lambda_)));
    }

    Wc_.clear();
    Wc_.push_back(lambda_ / (nx_ + lambda_) + (1.0 - pow(ALPHA, 2) + BETA));
    for (int i=1; i < 2 * nx_ + 1; i++){
        Wc_.push_back(1.0 / (2.0 * (nx_ + lambda_)));
    }
}

// ============================================================================
// SIGMA POINT GENERATION
// ============================================================================

/**
 * @brief Generate sigma points from mean and covariance
 * 
 * STUDENT TODO:
 * 1. Start with the mean as the first sigma point
 * 2. Compute Cholesky decomposition of covariance
 * 3. Generate 2*n symmetric sigma points around the mean
 */
std::vector<Eigen::VectorXd> UKF::generateSigmaPoints(const Eigen::VectorXd& mean,
                                                       const Eigen::MatrixXd& cov) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================

    // Cholesky decomposition
    Eigen::LLT<Eigen::MatrixXd> llt(cov);
    Eigen::MatrixXd L = llt.matrixL();
    Eigen::MatrixXd A = gamma_ * L;                                                       
    
    // Building sigma points
    std::vector<Eigen::VectorXd> sigma_points;
    sigma_points.resize(2 * nx_ + 1);
    sigma_points[0] = mean;
    for (int i = 0; i < nx_; i++){
        sigma_points[i + 1] = mean + A.col(i);
        sigma_points[i + 1 + nx_] = mean - A.col(i);
    } 

    
    return sigma_points;
}

// ============================================================================
// PROCESS MODEL
// ============================================================================

/**
 * @brief Apply motion model to a state vector
 * 
 * STUDENT TODO:
 * 1. Updates position: x' = x + dx, y' = y + dy
 * 2. Updates orientation: theta' = theta + dtheta (normalized)
 * 3. Updates velocities: vx' = dx/dt, vy' = dy/dt
 */
Eigen::VectorXd UKF::processModel(const Eigen::VectorXd& state, double dt,
                                  double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    Eigen::VectorXd new_state = state;
    new_state[0] += dx;
    new_state[1] += dy;
    new_state[2] += dtheta;
    new_state[2] = normalizeAngle(new_state[2]);
    if (dt > 1e-9) {
        new_state[3] = dx / dt;
        new_state[4] = dy / dt;
    }
    
    return new_state;
}

// ============================================================================
// MEASUREMENT MODEL
// ============================================================================

/**
 * @brief Predict measurement given current state and landmark
 * 
 * STUDENT TODO:
 * 1. Calculate relative position: landmark - robot position
 * 2. Transform to robot frame using robot orientation
 * 3. Return relative position in robot frame
 */
Eigen::Vector2d UKF::measurementModel(const Eigen::VectorXd& state, int landmark_id) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    
    auto it = landmarks_.find(landmark_id);

    if (it == landmarks_.end()) {
        return Eigen::Vector2d::Zero();
    }
    
    const double l_x = it->second.first;
    const double l_y = it->second.second;

    const double x = state(0);
    const double y = state(1);
    const double theta = state(2);

    Eigen::Vector2d rel_world;
    rel_world << (l_x - x), (l_y - y);

    
    Eigen::Rotation2D<double> rot(-theta);

    Eigen::Vector2d rel_robot = rot * rel_world;
    return rel_robot;
}

// ============================================================================
// ANGLE NORMALIZATION
// ============================================================================

double UKF::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ============================================================================
// PREDICTION STEP
// ============================================================================

/**
 * @brief Kalman Filter Prediction Step (Time Update)
 * 
 * STUDENT TODO:
 * 1. Generate sigma points from current state and covariance
 * 2. Propagate each sigma point through motion model
 * 3. Calculate mean and covariance of predicted sigma points
 * 4. Add process noise
 * 5. Update state and covariance estimates
 */
void UKF::predict(double dt, double dx, double dy, double dtheta) {
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    // 1. Sigma-Punkte generieren
    std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);
    const int n_sig = static_cast<int>(sigma_points.size());

    // 2. Sigma-Punkte durch Prozessmodell propagieren
    std::vector<Eigen::VectorXd> sigma_points_pred(n_sig);
    for (int i = 0; i < n_sig; ++i) {
        sigma_points_pred[i] = processModel(sigma_points[i], dt, dx, dy, dtheta);
    }

    // 3. neuen Zustandsmittelwert berechnen
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(nx_);
    for (int i = 0; i < n_sig; ++i) {
        x_pred += Wm_[i] * sigma_points_pred[i];
    }

    // Winkelanteil normalisieren (Index 2 = theta)
    x_pred(2) = normalizeAngle(x_pred(2));

    // 4. neue Kovarianz berechnen
    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(nx_, nx_);
    for (int i = 0; i < n_sig; ++i) {
        Eigen::VectorXd diff = sigma_points_pred[i] - x_pred;

        // Winkel normalisieren
        diff(2) = normalizeAngle(diff(2));

        P_pred += Wc_[i] * diff * diff.transpose();
    }

    // 5. Prozessrauschen hinzufügen
    P_pred += Q_;

    // 6. Zustand und Kovarianz aktualisieren
    x_ = x_pred;
    P_ = P_pred;
}

// ============================================================================
// UPDATE STEP
// ============================================================================

/**
 * @brief Kalman Filter Update Step (Measurement Update)
 * 
 * STUDENT TODO:
 * 1. Generate sigma points
 * 2. Transform through measurement model
 * 3. Calculate predicted measurement mean
 * 4. Calculate measurement and cross-covariance
 * 5. Compute Kalman gain
 * 6. Update state with innovation
 * 7. Update covariance
 */
void UKF::update(const std::vector<std::tuple<int, double, double, double>>& landmark_observations) {
    if (landmark_observations.empty()) {
        return;
    }
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    const int n_x  = nx_;  // 5
    const int n_z  = nz_;  // 2

    // Jede Landmark-Beobachtung einzeln verarbeiten
    for (const auto& obs : landmark_observations) {
        int    id        = std::get<0>(obs);
        double z_x       = std::get<1>(obs); // gemessene x-Position im Roboterframe
        double z_y       = std::get<2>(obs); // gemessene y-Position im Roboterframe
        double noise_cov = std::get<3>(obs); // zusätzl. Messunsicherheit (Skalar)

        if (!hasLandmark(id)) {
            continue;
        }

        // 1. tatsächliche Messung
        Eigen::Vector2d z;
        z << z_x, z_y;

        // 2. Messrauschkovarianz bauen
        Eigen::Matrix2d R = R_;
        R(0,0) += noise_cov;
        R(1,1) += noise_cov;

        // 3. Sigma-Punkte um aktuellen Zustand generieren
        std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);
        const int n_sig = static_cast<int>(sigma_points.size());

        // 4. Sigma-Punkte in den Messraum transformieren
        Eigen::MatrixXd Zsig(n_z, n_sig);
        for (int i = 0; i < n_sig; ++i) {
            Eigen::Vector2d z_i = measurementModel(sigma_points[i], id);
            Zsig.col(i) = z_i;
        }

        // 5. vorhergesagte Messung z_pred
        Eigen::Vector2d z_pred = Eigen::Vector2d::Zero();
        for (int i = 0; i < n_sig; ++i) {
            z_pred += Wm_[i] * Zsig.col(i);
        }

        // 6. Messkovarianz S
        Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
        for (int i = 0; i < n_sig; ++i) {
            Eigen::Vector2d z_diff = Zsig.col(i) - z_pred;
            // (hier kein Winkel; sonst: normalizeAngle)
            S += Wc_[i] * z_diff * z_diff.transpose();
        }
        S += R;

        // 7. Kreuzkovarianz P_xz
        Eigen::MatrixXd P_xz = Eigen::MatrixXd::Zero(n_x, n_z);
        for (int i = 0; i < n_sig; ++i) {
            Eigen::VectorXd x_diff = sigma_points[i] - x_;
            x_diff(2) = normalizeAngle(x_diff(2));

            Eigen::Vector2d z_diff = Zsig.col(i) - z_pred;
            P_xz += Wc_[i] * x_diff * z_diff.transpose();
        }

        // 8. Kalman-Gain
        Eigen::MatrixXd K = P_xz * S.inverse();

        // 9. Innovation
        Eigen::Vector2d z_diff = z - z_pred;

        // 10. Zustand und Kovarianz updaten
        x_ = x_ + K * z_diff;
        x_(2) = normalizeAngle(x_(2));

        P_ = P_ - K * S * K.transpose();
    }
    
}

// ============================================================================
// LANDMARK MANAGEMENT
// ============================================================================

void UKF::setLandmarks(const std::map<int, std::pair<double, double>>& landmarks) {
    landmarks_ = landmarks;
}

bool UKF::hasLandmark(int id) const {
    return landmarks_.find(id) != landmarks_.end();
}
