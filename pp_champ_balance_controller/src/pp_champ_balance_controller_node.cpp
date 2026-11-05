#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <string>

namespace {
constexpr double kPi = 3.14159265358979323846;
double clampValue(double v, double limit) {
  return std::max(-limit, std::min(limit, v));
}
bool toRPY(const geometry_msgs::Quaternion& qmsg,
           double& roll, double& pitch, double& yaw) {
  if (!std::isfinite(qmsg.x) || !std::isfinite(qmsg.y) ||
      !std::isfinite(qmsg.z) || !std::isfinite(qmsg.w)) return false;
  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  const double n2 = q.length2();
  if (!std::isfinite(n2) || n2 < 1e-12) return false;
  q.normalize();
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
}
}  // namespace

class BalanceController {
public:
  BalanceController() : nh_(), pnh_("~"), has_manual_(false), has_imu_(false),
                        prev_correction_roll_(0.0), prev_correction_pitch_(0.0),
                        last_output_initialized_(false) {
    pnh_.param<std::string>("input_topic", input_topic_, "body_pose_manual");
    pnh_.param<std::string>("output_topic", output_topic_, "body_pose");
    pnh_.param<std::string>("imu_topic", imu_topic_, "imu/data");
    pnh_.param("enable_balance", enable_balance_, false);
    pnh_.param("enable_pd_debug", enable_pd_debug_, false);  // Periodic INFO only
    pnh_.param("kp_roll", kp_roll_, 0.3);
    pnh_.param("kp_pitch", kp_pitch_, 0.3);
    pnh_.param("kd_roll", kd_roll_, 0.0);
    pnh_.param("kd_pitch", kd_pitch_, 0.0);
    pnh_.param("max_correction_deg", max_correction_deg_, 10.0);
    pnh_.param("max_correction_rate_deg_s", max_correction_rate_deg_s_, 20.0);
    pnh_.param("imu_timeout_sec", imu_timeout_sec_, 0.2);
    pnh_.param("control_rate_hz", control_rate_hz_, 50.0);
    if (control_rate_hz_ <= 0 || imu_timeout_sec_ <= 0 ||
        max_correction_deg_ < 0 || max_correction_rate_deg_s_ <= 0 ||
        !std::isfinite(kp_roll_) || !std::isfinite(kp_pitch_) ||
        !std::isfinite(kd_roll_) || !std::isfinite(kd_pitch_) ||
        kp_roll_ < 0 || kp_pitch_ < 0 || kd_roll_ < 0 || kd_pitch_ < 0) {
      ROS_FATAL("Invalid balance parameters; check config/balance.yaml");
      ros::shutdown();
      return;
    }
    if (nh_.resolveName(input_topic_) == nh_.resolveName(output_topic_)) {
      ROS_FATAL("Input and output topics must differ to avoid a relay feedback loop");
      ros::shutdown();
      return;
    }
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>(output_topic_, 1);
    pose_sub_ = nh_.subscribe(input_topic_, 1, &BalanceController::poseCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 1, &BalanceController::imuCallback, this);
    if (enable_balance_) {
      timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
                               &BalanceController::controlCallback, this);
    }
    ROS_INFO_STREAM("balance_controller: " << nh_.resolveName(input_topic_) << " -> "
                    << nh_.resolveName(output_topic_) << ", imu="
                    << nh_.resolveName(imu_topic_) << ", enable_balance=" << enable_balance_);
    ROS_INFO_STREAM("PD startup: Kp_roll=" << kp_roll_ << " Kd_roll=" << kd_roll_
                    << " Kp_pitch=" << kp_pitch_ << " Kd_pitch=" << kd_pitch_);
    if (!enable_balance_) {
      ROS_WARN("Balance disabled: manual pose relay only; PD control is NOT running");
    }
  }
private:
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    manual_pose_ = *msg;
    has_manual_ = true;
    // Exactly preserve the tested stage-1 behavior when balancing is disabled.
    if (!enable_balance_) pose_pub_.publish(*msg);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // -1 means orientation is not supplied by the sensor.
    if (msg->orientation_covariance[0] == -1.0) return;
    double yaw = 0.0;
    if (!toRPY(msg->orientation, imu_roll_, imu_pitch_, yaw)) {
      ROS_WARN_THROTTLE(2.0, "Invalid IMU orientation quaternion");
      return;
    }
    // Angular velocity is in rad/s (body axes). Only require valid rates
    // when D feedback is enabled; kd=0 preserves the original P behavior.
    if (kd_roll_ > 0.0 && !std::isfinite(msg->angular_velocity.x)) {
      ROS_WARN_THROTTLE(2.0, "Invalid IMU roll angular velocity");
      return;
    }
    if (kd_pitch_ > 0.0 && !std::isfinite(msg->angular_velocity.y)) {
      ROS_WARN_THROTTLE(2.0, "Invalid IMU pitch angular velocity");
      return;
    }
    imu_gyro_x_ = (kd_roll_ > 0.0) ? msg->angular_velocity.x : 0.0;
    imu_gyro_y_ = (kd_pitch_ > 0.0) ? msg->angular_velocity.y : 0.0;
    has_imu_ = true;
    last_imu_receive_time_ = ros::Time::now();
  }

  void controlCallback(const ros::TimerEvent& event) {
    if (!has_manual_) {
      ROS_WARN_THROTTLE(2.0, "PD waiting: no manual body_pose received; no control output");
      return;  // Do not invent a body height or neutral pose.
    }
    const ros::Time now = ros::Time::now();
    const bool imu_fresh = has_imu_ &&
        (now - last_imu_receive_time_).toSec() >= 0.0 &&
        (now - last_imu_receive_time_).toSec() <= imu_timeout_sec_;

    // Manual commands are passed through unchanged if IMU is missing/stale.
    if (!imu_fresh) {
      ROS_WARN_THROTTLE(2.0, "IMU unavailable/stale: passing through manual pose");
      prev_correction_roll_ = prev_correction_pitch_ = 0.0;
      last_output_initialized_ = false;
      pose_pub_.publish(manual_pose_);
      return;
    }

    double manual_roll = 0, manual_pitch = 0, manual_yaw = 0;
    if (!toRPY(manual_pose_.orientation, manual_roll, manual_pitch, manual_yaw)) {
      ROS_WARN_THROTTLE(2.0, "Invalid manual pose quaternion: skipping output");
      return;
    }

    // PD feedback: derivative on measurement avoids a derivative kick
    // when manual target orientation changes. For near-level attitude,
    // body angular velocity x/y approximates roll/pitch Euler rates.
    // Kp is dimensionless; Kd has units of seconds.
    // The negative gyro term opposes measured rotation.
    const double limit = max_correction_deg_ * kPi / 180.0;
    const double p_roll = kp_roll_ * (manual_roll - imu_roll_);
    const double d_roll = -kd_roll_ * imu_gyro_x_;
    const double p_pitch = kp_pitch_ * (manual_pitch - imu_pitch_);
    const double d_pitch = -kd_pitch_ * imu_gyro_y_;
    const double roll_desired = clampValue(p_roll + d_roll, limit);
    const double pitch_desired = clampValue(p_pitch + d_pitch, limit);
    double dt = (event.current_real - event.last_real).toSec();
    if (!last_output_initialized_) {
      prev_correction_roll_ = 0.0;
      prev_correction_pitch_ = 0.0;
      last_output_initialized_ = true;
      dt = 1.0 / control_rate_hz_;
    }
    dt = std::max(0.0, std::min(dt, 0.1));
    const double max_step = max_correction_rate_deg_s_ * kPi / 180.0 * dt;
    const double corr_roll = prev_correction_roll_ +
                             clampValue(roll_desired - prev_correction_roll_, max_step);
    const double corr_pitch = prev_correction_pitch_ +
                              clampValue(pitch_desired - prev_correction_pitch_, max_step);
    prev_correction_roll_ = corr_roll;
    prev_correction_pitch_ = corr_pitch;

    geometry_msgs::Pose output = manual_pose_;
    tf2::Quaternion q;
    q.setRPY(manual_roll + corr_roll, manual_pitch + corr_pitch, manual_yaw);
    q.normalize();
    output.orientation.x = q.x();
    output.orientation.y = q.y();
    output.orientation.z = q.z();
    output.orientation.w = q.w();
    pose_pub_.publish(output);

    // Runtime diagnostic only. Startup INFO/WARN messages are unaffected.
    // 1 Hz logging avoids flooding the console at the 50 Hz control rate.
    if (enable_pd_debug_) {
      constexpr double kRadToDeg = 180.0 / kPi;
      ROS_INFO_STREAM_THROTTLE(1.0,
          "PD ROLL active: Kp=" << kp_roll_ << " Kd=" << kd_roll_
          << " gyro=" << imu_gyro_x_ << "rad/s"
          << " P=" << p_roll * kRadToDeg << "deg"
          << " D=" << d_roll * kRadToDeg << "deg"
          << " desired=" << roll_desired * kRadToDeg << "deg"
          << " applied=" << corr_roll * kRadToDeg << "deg");
      ROS_INFO_STREAM_THROTTLE(1.0,
          "PD PITCH active: Kp=" << kp_pitch_ << " Kd=" << kd_pitch_
          << " gyro=" << imu_gyro_y_ << "rad/s"
          << " P=" << p_pitch * kRadToDeg << "deg"
          << " D=" << d_pitch * kRadToDeg << "deg"
          << " desired=" << pitch_desired * kRadToDeg << "deg"
          << " applied=" << corr_pitch * kRadToDeg << "deg");
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher pose_pub_;
  ros::Subscriber pose_sub_, imu_sub_;
  ros::Timer timer_;
  std::string input_topic_, output_topic_, imu_topic_;
  bool enable_balance_, enable_pd_debug_, has_manual_, has_imu_;
  double kp_roll_, kp_pitch_, kd_roll_, kd_pitch_;
  double max_correction_deg_, max_correction_rate_deg_s_;
  double imu_timeout_sec_, control_rate_hz_;
  double imu_roll_ = 0.0, imu_pitch_ = 0.0;
  double imu_gyro_x_ = 0.0, imu_gyro_y_ = 0.0;
  double prev_correction_roll_, prev_correction_pitch_;
  bool last_output_initialized_;
  ros::Time last_imu_receive_time_;
  geometry_msgs::Pose manual_pose_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pp_champ_balance_controller");
  BalanceController controller;
  ros::spin();
  return 0;
}
