#include "kalman_positioning/ukf.hpp"
#include <iostream>
#include <map>

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
    
    lambda_ = ALPHA * ALPHA * (nx_ + KAPPA) - nx_;
      
    // 2. Initialize state vector x_ with zeros 
    x_ = Eigen::VectorXd::Zero(nx_);
kappa_
    // 3. Initialize state covariance matrix P_ 
    P_ = Eigen::MatrixXd::Identity(nx_, nx_);

    // 4. Set process noise covariance Q_
    Q_ = Eigen::MatrixXd::Zero(nx_, nx_);
    Q_(0, 0) = process_noise_xy;
    Q_(1, 1) = process_noise_xy;
    Q_(2, 2) = process_noise_theta;
    Q_(3, 3) = 0.0;
    Q_(4, 4) = 0.0;

    // 5. Set measurement noise covariance R_
    R_ = Eigen::MatrixXd::Zero(nz_, nz_);
    R_(0, 0) = measurement_noise_xy;
    R_(1, 1) = measurement_noise_xy;

    // 6. Calculate sigma point weights for mean and covariance
    Wm_ = Eigen::VectorXd(2 * nx_ + 1);
    Wc_ = Eigen::VectorXd(2 * nx_ + 1);

    Wm_(0) = lambda_ / (nx_ + lambda_);
    
    Wc_(0) = Wm_(0) + (1 - ALPHA * ALPHA + BETA);

    double weight = 0.5 * lambda_ / (nx_ + lambda_);
    for (int i = 1; i < 2 * nx_ + 1; ++i) {
        Wm_(i) = weight;
        Wc_(i) = weight;
    } 
    std::cout << "UKF Constructor: TODO - Implement filter initialization" << std::endl;
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

    std::vector<Eigen::VectorXd> sigma_points;

    sigma_points.reserve(2 * nx_ + 1);

    // 1. Start with the mean as the first sigma point (X_0)
    sigma_points.push_back(mean);

    // 2. Compute Cholesky decomposition of covariance matrix P
    Eigen::LLT<Eigen::MatrixXd> lltOfP(cov);
    Eigen::MatrixXd L = lltOfP.matrixL();

    double gamma = std::sqrt(lambda_ + nx_);

    // 3. Generate 2*n symmetric sigma points around the mean
    for (int i = 0; i < nx_; ++i) {
        Eigen::VectorXd point = mean + (gamma * L.col(i));
        sigma_points.push_back(point);
    }
    for (int i = 0; i < nx_; ++i) {
        Eigen::VectorXd point = mean - (gamma * L.col(i));
        sigma_points.push_back(point);
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

    // 1. Updates position: x' = x + dx, y' = y + dy
    new_state(0) += dx;
    new_state(1) += dy;

    // 2. Updates orientation: theta' = theta + dtheta (normalized)
    double new_theta = new_state(2) + dtheta;
    new_state(2) = normalizeAngle(new_theta);

    // 3. Updates velocities: vx' = dx/dt, vy' = dy/dt
    new_state(3) = dx / dt; 
    new_state(4) = dy / dt;                                  
    
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
    
    if (landmarks_.find(landmark_id) == landmarks_.end()) {
        return Eigen::Vector2d::Zero();
    }

    Eigen::Vector2d landmark = landmarks_.at(landmark_id);
    double l_x = landmark(0);
    double l_y = landmark(1);

    double p_x = state(0);
    double p_y = state(1);
    double theta = state(2);

    // 1. Calculate relative position: landmark - robot position (Delta in Global Frame)
    double dx = l_x - p_x;
    double dy = l_y - p_y;

    // 2. Transform to robot frame using robot orientation
    double x_robot = std::cos(theta) * dx + std::sin(theta) * dy;
    double y_robot = -std::sin(theta) * dx + std::cos(theta) * dy;

    // 3. Return relative position in robot frame
    Eigen::Vector2d relative_pos;
    relative_pos(0) = x_robot;
    relative_pos(1) = y_robot;
    
    return relative_pos;
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
    
    
    // 1. Generate sigma points from current state and covariance
    std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);

    // 2. Propagate each sigma point through the motion model
    std::vector<Eigen::VectorXd> sigma_points_pred;
    sigma_points_pred.reserve(2 * nx_ + 1);

    for (const auto& sp : sigma_points) {
        Eigen::VectorXd sp_pred = processModel(sp, dt, dx, dy, dtheta);
        sigma_points_pred.push_back(sp_pred);
    }

    // 3. Calculate mean and covariance of predicted sigma points 
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(nx_);
    
    for (int i = 0; i < 2 * nx_ + 1; ++i) {
        x_pred += Wm_(i) * sigma_points_pred[i];
    }

    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(nx_, nx_);

    for (int i = 0; i < 2 * nx_ + 1; ++i) {
        // State difference
        Eigen::VectorXd x_diff = sigma_points_pred[i] - x_pred;

        x_diff(2) = normalizeAngle(x_diff(2));

        P_pred += Wc_(i) * x_diff * x_diff.transpose();
    }

    // 4. Add process noise covariance Q
    P_pred += Q_;

    // 5. Update state and covariance estimates
    x_ = x_pred;
    P_ = P_pred;

    std::cout << "UKF Predict: TODO - Implement prediction step" << std::endl;
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
    
    
    // STUDENT IMPLEMENTATION STARTS HERE
    // ========================================================================
    for (const auto& observation : landmark_observations) {
        
        int landmark_id = std::get<0>(observation);
        double meas_x = std::get<1>(observation);
        double meas_y = std::get<2>(observation);

        Eigen::Vector2d z;
        z(0) = meas_x;
        z(1) = meas_y;

        // 1. Generate sigma points
        std::vector<Eigen::VectorXd> sigma_points = generateSigmaPoints(x_, P_);

        // 2. Transform through measurement model
        int n_sig = 2 * nx_ + 1;
        std::vector<Eigen::VectorXd> sigma_points_pred; 
        sigma_points_pred.reserve(n_sig);

        for (const auto& sig_point : sigma_points) {
            Eigen::Vector2d sigma_predicted_point = measurementModel(sig_point, landmark_id);
            sigma_points_pred.push_back(sigma_predicted_point);
        }

        // 3. Calculate predicted measurement mean
        Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(nz_);
        for (int i = 0; i < n_sig; ++i) {
            z_pred += Wm_(i) * sigma_points_pred[i];
        }

        // 4. Calculate measurement covariance (S) and cross-covariance (Tc)
        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nz_, nz_);
        Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(nx_, nz_);

        for (int i = 0; i < n_sig; ++i) {
            Eigen::VectorXd z_diff = sigma_points_pred[i] - z_pred;
            
            Eigen::VectorXd x_diff = sigma_points[i] - x_;

            x_diff(2) = normalizeAngle(x_diff(2));
            
            S += Wc_(i) * z_diff * z_diff.transpose();
            Tc += Wc_(i) * x_diff * z_diff.transpose();
        }

        S += R_;

        // 5. Compute Kalman gain
        Eigen::MatrixXd K = Tc * S.inverse();

        // 6. Update state with innovation
        Eigen::VectorXd z_diff = z - z_pred;
        
        x_ += K * z_diff;

        // 7. Update covariance
        P_ -= K * S * K.transpose();
    }
    
    std::cout << "UKF Update: TODO - Implement measurement update step" << std::endl;
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
