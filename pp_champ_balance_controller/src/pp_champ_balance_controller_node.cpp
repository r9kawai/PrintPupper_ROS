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
    pnh_.param("kp_roll", kp_roll_, 0.3);
    pnh_.param("kp_pitch", kp_pitch_, 0.3);
    pnh_.param("max_correction_deg", max_correction_deg_, 10.0);
    pnh_.param("max_correction_rate_deg_s", max_correction_rate_deg_s_, 20.0);
    pnh_.param("imu_timeout_sec", imu_timeout_sec_, 0.2);
    pnh_.param("control_rate_hz", control_rate_hz_, 50.0);
    if (control_rate_hz_ <= 0 || imu_timeout_sec_ <= 0 ||
        max_correction_deg_ < 0 || max_correction_rate_deg_s_ <= 0 ||
        kp_roll_ < 0 || kp_pitch_ < 0) {
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
    has_imu_ = true;
    last_imu_receive_time_ = ros::Time::now();
  }

  void controlCallback(const ros::TimerEvent& event) {
    if (!has_manual_) return;  // Do not invent a body height or neutral pose.
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

    // Preliminary P feedback; polarity must be confirmed with servos disabled.
    const double limit = max_correction_deg_ * kPi / 180.0;
    const double roll_desired = clampValue(kp_roll_ * (manual_roll - imu_roll_), limit);
    const double pitch_desired = clampValue(kp_pitch_ * (manual_pitch - imu_pitch_), limit);
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
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher pose_pub_;
  ros::Subscriber pose_sub_, imu_sub_;
  ros::Timer timer_;
  std::string input_topic_, output_topic_, imu_topic_;
  bool enable_balance_, has_manual_, has_imu_;
  double kp_roll_, kp_pitch_, max_correction_deg_, max_correction_rate_deg_s_;
  double imu_timeout_sec_, control_rate_hz_;
  double imu_roll_ = 0.0, imu_pitch_ = 0.0;
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
