#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <memory>
#include <cmath>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kalman_positioning/ukf.hpp"
#include "kalman_positioning/landmark_manager.hpp"



/**
 * @brief Positioning node for UKF-based robot localization (Student Assignment)
 * 
 * This node subscribes to:
 *   - /robot_noisy: Noisy odometry (dead-reckoning)
 *   - /landmarks_observed: Noisy landmark observations
 * 
 * And publishes to:
 *   - /robot_estimated_odometry: Estimated pose and velocity from filter
 * 
 * STUDENT ASSIGNMENT:
 * Implement the Kalman filter logic to fuse odometry and landmark observations
 * to estimate the robot's true position.
 */
class PositioningNode : public rclcpp::Node {
public:
    PositioningNode() : Node("kalman_positioning_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing Kalman Positioning Node");
        
        // Create subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot_noisy",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::odometryCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_noisy");
        
        landmarks_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/landmarks_observed",
            rclcpp::QoS(10),
            std::bind(&PositioningNode::landmarksObservedCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to /landmarks_observed");
        
        // Create publisher
        estimated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/robot_estimated_odometry", rclcpp::QoS(10)
        );
        RCLCPP_INFO(this->get_logger(), "Publishing to /robot_estimated_odometry");
        
        RCLCPP_INFO(this->get_logger(), "Kalman Positioning Node initialized successfully");

        std::string csv_path = this->get_parameter("landmarks_csv_path").as_string();

        // 2. Load Landmarks
        if (landmark_manager_.loadFromCSV(csv_path)) {
            // 3. Pass to UKF
            ukf_.setLandmarks(landmark_manager_.getLandmarks());
            RCLCPP_INFO(this->get_logger(), "Landmarks loaded into UKF successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load landmarks into UKF!");
        }
    }

private:
    // ============================================================================
    // SUBSCRIBERS AND PUBLISHERS
    // ============================================================================
    
    // ==========================
    // UKF FILTER
    // ==========================

    UKF ukf_{0.01, 0.01, 0.01, 10};// process_noise_xy, process_noise_theta, measurement_noise_xy, num_landmarks
    rclcpp::Time last_odom_time_;   // für dt Berechnung
    double last_theta_;             // für dtheta Berechnung


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_obs_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
    
    // ============================================================================
    // PLACEHOLDER: KALMAN FILTER STATE
    // ============================================================================
    // Students should implement a proper Kalman filter (e.g., UKF, EKF) 
    // with the following state:
    //   - Position: x, y (m)
    //   - Orientation: theta (rad)
    //   - Velocity: vx, vy (m/s)
    // And maintain:
    //   - State covariance matrix
    //   - Process noise covariance
    //   - Measurement noise covariance
    
    // ============================================================================
    // CALLBACK FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Callback for noisy odometry measurements
     * 
     * STUDENT TODO:
     * 1. Extract position (x, y) and orientation (theta) from the message
     * 2. Update the Kalman filter's prediction step with this odometry
     * 3. Publish the estimated odometry
     */
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Odometry received: x=%.3f, y=%.3f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);
        
        // STUDENT ASSIGNMENT STARTS HERE
        // ========================================================================
        
        // Placeholder: Extract and log the data
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = quaternionToYaw(msg->pose.pose.orientation);
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        
        //first callback
        if (last_odom_time_.nanoseconds() == 0){
            last_odom_time_ = msg->header.stamp;
            last_theta_ = theta;
            publishEstimatedOdometry(msg->header.stamp, *msg);
            return;
        }


        double dt = (rclcpp::Time(msg->header.stamp)- last_odom_time_).seconds();
        //double dt = (msg->header.stamp - last_odom_time_).seconds();

        // Verschiebung berechnen
        double dx = vx * dt;
        double dy = vy * dt;
        double dtheta = normalizeAngle(theta - last_theta_);

        // UKF Prediction
        ukf_.predict(dt, dx, dy, dtheta);

        // Update letzte Messwerte
        last_odom_time_ = msg->header.stamp;
        last_theta_ = theta;

        // Publiziere geschätzte Odometry
        nav_msgs::msg::Odometry est_msg;
        est_msg.header.stamp = msg->header.stamp;
        est_msg.header.frame_id = "map";
        est_msg.child_frame_id = "robot_estimated";

        est_msg.pose.pose.position.x = ukf_.getState()(0);
        est_msg.pose.pose.position.y = ukf_.getState()(1);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, ukf_.getState()(2));
        est_msg.pose.pose.orientation = tf2::toMsg(q);

        est_msg.twist.twist.linear.x = ukf_.getState()(3);
        est_msg.twist.twist.linear.y = ukf_.getState()(4);
            
    
        /** 
        RCLCPP_DEBUG(this->get_logger(), 
            "Parsed: x=%.3f, y=%.3f, theta=%.3f, vx=%.3f, vy=%.3f",
            x, y, theta, vx, vy);
        */
        // For now, publish the noisy odometry as estimated
        // Students should replace this with actual filter output


        publishEstimatedOdometry(msg->header.stamp, est_msg);
    }
    
    /**
     * @brief Callback for landmark observations
     * 
     * STUDENT TODO:
     * 1. Parse the PointCloud2 data to extract landmark observations
     * 2. Update the Kalman filter's measurement update step with these observations
     * 3. Optionally publish the updated estimated odometry
     */
    void landmarksObservedCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // STUDENT ASSIGNMENT STARTS HERE
        // ========================================================================
        

        /**RCLCPP_DEBUG(this->get_logger(), 
           Landmark observation received with %lu points", msg->width);
        */

        //Define observation
        std::vector<std::tuple<int, double, double, double>> observations;

        
        // Placeholder: Parse and log the observations
        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_id(*msg, "id");
            
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_id) {
                int landmark_id = static_cast<int>(*iter_id);
                float obs_x = *iter_x;
                float obs_y = *iter_y;
                double noise_cov = 0.01; // Beispiel: Messrauschen, kann angepasst werden

                observations.push_back(std::make_tuple(landmark_id, obs_x, obs_y, noise_cov));
                
            }
            // UKF Measurement Update
            ukf_.update(observations);

            //Publiziere aktualisierte geschätzte Odometry
            nav_msgs::msg::Odometry est_msg;
            est_msg.header.stamp = msg->header.stamp;
            est_msg.header.frame_id = "map";
            est_msg.child_frame_id = "robot_estimated";

            est_msg.pose.pose.position.x = ukf_.getState()(0);
            est_msg.pose.pose.position.y = ukf_.getState()(1);
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, ukf_.getState()(2));
            est_msg.pose.pose.orientation = tf2::toMsg(q);

            est_msg.twist.twist.linear.x = ukf_.getState()(3);
            est_msg.twist.twist.linear.y = ukf_.getState()(4);

            publishEstimatedOdometry(msg->header.stamp, est_msg);

            /** 
            RCLCPP_DEBUG(this->get_logger(), 
                "Processed %d landmark observations", count);
            */
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to parse landmark observations: %s", e.what());
        }
    }
    
    // ============================================================================
    // HELPER FUNCTIONS
    // ============================================================================
    
    /**
     * @brief Convert quaternion to yaw angle
     * @param q Quaternion from orientation
     * @return Yaw angle in radians [-pi, pi]
     */
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Input angle in radians
     * @return Normalized angle in [-pi, pi]
     */
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    /**
     * @brief Publish estimated odometry message
     * @param timestamp Message timestamp
     * @param odom_msg Odometry message to publish
     */
    void publishEstimatedOdometry(const rclcpp::Time& timestamp, 
                                  const nav_msgs::msg::Odometry& odom_msg) {

        nav_msgs::msg::Odometry est_msg;
        est_msg.header.stamp = timestamp;
        est_msg.header.frame_id = "map";
        est_msg.child_frame_id = "robot_estimated";

        est_msg.pose.pose.position.x = ukf_.getState()(0);
        est_msg.pose.pose.position.y = ukf_.getState()(1);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, ukf_.getState()(2));
        est_msg.pose.pose.orientation = tf2::toMsg(q);

        est_msg.twist.twist.linear.x = ukf_.getState()(3);
        est_msg.twist.twist.linear.y = ukf_.getState()(4);

        estimated_odom_pub_->publish(est_msg);
        // STUDENT TODO: Replace this with actual filter output
        // Set position, orientation, velocity, and covariance from your Kalman filter
        // Currently using placeholder values (noisy odometry)
        
        
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositioningNode>());
    rclcpp::shutdown();
    return 0;
}
