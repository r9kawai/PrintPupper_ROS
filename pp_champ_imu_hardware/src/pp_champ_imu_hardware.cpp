#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

// ============================================================
// User configuration
// ============================================================

constexpr const char* I2C_DEVICE = "/dev/i2c-1";
constexpr uint8_t MPU6050_ADDRESS = 0x68;

// Processing rate
constexpr float IMU_READ_RATE_HZ = 100.0f;
constexpr float IMU_PUBLISH_RATE_HZ = 50.0f;

// Complementary filter
constexpr float COMPLEMENTARY_ALPHA = 0.98f;

// true  : integrate gyro Z as yaw
// false : yaw is always zero
constexpr bool ENABLE_YAW_INTEGRATION = true;

// MPU6050 configuration
//
// Accelerometer : +/-2 g
// Gyroscope     : +/-250 deg/s
//
constexpr float ACCEL_SCALE = 16384.0f;  // LSB/g
constexpr float GYRO_SCALE  = 131.0f;    // LSB/(deg/s)

constexpr float GRAVITY = 9.80665f;
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;

// Startup gyro calibration
constexpr bool ENABLE_GYRO_CALIBRATION = true;
constexpr int GYRO_CALIBRATION_SAMPLES = 500;

// ROS frame name
constexpr const char* IMU_FRAME_ID = "base_link";

// ============================================================
// MPU6050 registers
// ============================================================

constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

// ============================================================
// Raw sensor structure
// ============================================================

struct MPU6050Raw
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temperature;

    int16_t gx;
    int16_t gy;
    int16_t gz;
};

// ============================================================
// Physical sensor values
// ============================================================

struct IMUData
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
};

// ============================================================
// MPU6050 class
// ============================================================

class MPU6050
{
public:

    MPU6050()
        : fd_(-1)
    {
    }

    ~MPU6050()
    {
        if (fd_ >= 0)
        {
            close(fd_);
        }
    }

    bool openDevice()
    {
        fd_ = open(I2C_DEVICE, O_RDWR);

        if (fd_ < 0)
        {
            std::cerr
                << "Failed to open "
                << I2C_DEVICE
                << std::endl;

            return false;
        }

        if (ioctl(fd_, I2C_SLAVE, MPU6050_ADDRESS) < 0)
        {
            std::cerr
                << "Failed to select MPU6050 I2C address 0x"
                << std::hex
                << static_cast<int>(MPU6050_ADDRESS)
                << std::dec
                << std::endl;

            close(fd_);
            fd_ = -1;

            return false;
        }

        return true;
    }

    bool initialize()
    {
        uint8_t who_am_i = 0;

        if (!readRegister(REG_WHO_AM_I, who_am_i))
        {
            std::cerr << "Failed to read WHO_AM_I" << std::endl;
            return false;
        }

        std::cout
            << "MPU6050 WHO_AM_I = 0x"
            << std::hex
            << static_cast<int>(who_am_i)
            << std::dec
            << std::endl;

        if (who_am_i != 0x68)
        {
            std::cerr
                << "Unexpected MPU6050 WHO_AM_I value."
                << std::endl;

            return false;
        }

        // Wake up MPU6050
        //
        // Clock source = PLL with X axis gyroscope
        //
        if (!writeRegister(REG_PWR_MGMT_1, 0x01))
            return false;

        usleep(100000);

        // Sample rate:
        //
        // Gyro output rate = 1 kHz with DLPF enabled
        //
        // SampleRate = 1000 / (1 + SMPLRT_DIV)
        //
        // 1000 / (1 + 9) = 100 Hz
        //
        if (!writeRegister(REG_SMPLRT_DIV, 9))
            return false;

        // DLPF_CFG = 3
        //
        // Moderate filtering suitable as initial setting.
        //
        if (!writeRegister(REG_CONFIG, 0x03))
            return false;

        // Gyroscope +/-250 deg/s
        if (!writeRegister(REG_GYRO_CONFIG, 0x00))
            return false;

        // Accelerometer +/-2 g
        if (!writeRegister(REG_ACCEL_CONFIG, 0x00))
            return false;

        return true;
    }

    bool readRaw(MPU6050Raw& data)
    {
        uint8_t buffer[14];

        uint8_t reg = REG_ACCEL_XOUT_H;

        // Set starting register
        if (write(fd_, &reg, 1) != 1)
        {
            return false;
        }

        // Burst read:
        //
        // AX AY AZ TEMP GX GY GZ
        //
        if (read(fd_, buffer, sizeof(buffer))
            != static_cast<ssize_t>(sizeof(buffer)))
        {
            return false;
        }

        data.ax = combine(buffer[0],  buffer[1]);
        data.ay = combine(buffer[2],  buffer[3]);
        data.az = combine(buffer[4],  buffer[5]);

        data.temperature =
            combine(buffer[6], buffer[7]);

        data.gx = combine(buffer[8],  buffer[9]);
        data.gy = combine(buffer[10], buffer[11]);
        data.gz = combine(buffer[12], buffer[13]);

        return true;
    }

private:

    int fd_;

    static int16_t combine(uint8_t high, uint8_t low)
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(high) << 8)
            | static_cast<uint16_t>(low));
    }

    bool writeRegister(uint8_t reg, uint8_t value)
    {
        uint8_t buffer[2];

        buffer[0] = reg;
        buffer[1] = value;

        return write(fd_, buffer, 2) == 2;
    }

    bool readRegister(uint8_t reg, uint8_t& value)
    {
        if (write(fd_, &reg, 1) != 1)
        {
            return false;
        }

        if (read(fd_, &value, 1) != 1)
        {
            return false;
        }

        return true;
    }
};

// ============================================================
// Attitude estimator
// ============================================================

class AttitudeEstimator
{
public:

    AttitudeEstimator()
        : roll_(0.0f),
          pitch_(0.0f),
          yaw_(0.0f),
          gyro_offset_x_(0.0f),
          gyro_offset_y_(0.0f),
          gyro_offset_z_(0.0f),
          initialized_(false)
    {
    }

    void setGyroOffset(float x, float y, float z)
    {
        gyro_offset_x_ = x;
        gyro_offset_y_ = y;
        gyro_offset_z_ = z;
    }

    void update(
        const IMUData& imu,
        float dt)
    {
        const float gx =
            imu.gx - gyro_offset_x_;

        const float gy =
            imu.gy - gyro_offset_y_;

        const float gz =
            imu.gz - gyro_offset_z_;

        // ----------------------------------------------------
        // Accelerometer attitude
        // ----------------------------------------------------

        const float roll_acc =
            std::atan2(imu.ay, imu.az);

        const float pitch_acc =
            std::atan2(
                -imu.ax,
                std::sqrt(
                    imu.ay * imu.ay +
                    imu.az * imu.az));

        // ----------------------------------------------------
        // Initial attitude
        // ----------------------------------------------------

        if (!initialized_)
        {
            roll_ = roll_acc;
            pitch_ = pitch_acc;
            yaw_ = 0.0f;

            initialized_ = true;

            return;
        }

        // ----------------------------------------------------
        // Complementary filter
        // ----------------------------------------------------

        roll_ =
            COMPLEMENTARY_ALPHA *
            (roll_ + gx * dt)
            +
            (1.0f - COMPLEMENTARY_ALPHA) *
            roll_acc;

        pitch_ =
            COMPLEMENTARY_ALPHA *
            (pitch_ + gy * dt)
            +
            (1.0f - COMPLEMENTARY_ALPHA) *
            pitch_acc;

        // ----------------------------------------------------
        // Yaw
        // ----------------------------------------------------

        if (ENABLE_YAW_INTEGRATION)
        {
            yaw_ += gz * dt;

            normalizeYaw();
        }
        else
        {
            yaw_ = 0.0f;
        }
    }

    float roll() const
    {
        return roll_;
    }

    float pitch() const
    {
        return pitch_;
    }

    float yaw() const
    {
        return yaw_;
    }

private:

    float roll_;
    float pitch_;
    float yaw_;

    float gyro_offset_x_;
    float gyro_offset_y_;
    float gyro_offset_z_;

    bool initialized_;

    void normalizeYaw()
    {
        while (yaw_ > PI)
        {
            yaw_ -= 2.0f * PI;
        }

        while (yaw_ < -PI)
        {
            yaw_ += 2.0f * PI;
        }
    }
};

// ============================================================
// Raw -> physical units
// ============================================================

IMUData convertRawData(const MPU6050Raw& raw)
{
    IMUData imu;
    IMUData imu_or;

    // Accelerometer -> m/s^2

    imu.ax =
        static_cast<float>(raw.ax)
        / ACCEL_SCALE
        * GRAVITY;

    imu.ay =
        static_cast<float>(raw.ay)
        / ACCEL_SCALE
        * GRAVITY;

    imu.az =
        static_cast<float>(raw.az)
        / ACCEL_SCALE
        * GRAVITY;

    // Gyroscope -> rad/s

    imu.gx =
        static_cast<float>(raw.gx)
        / GYRO_SCALE
        * DEG_TO_RAD;

    imu.gy =
        static_cast<float>(raw.gy)
        / GYRO_SCALE
        * DEG_TO_RAD;

    imu.gz =
        static_cast<float>(raw.gz)
        / GYRO_SCALE
        * DEG_TO_RAD;

    // MPU6050 raw -> ROS base_link as PrintPupper design
    imu_or.ax =  imu.ay;
    imu_or.ay = -imu.ax;
    imu_or.az =  imu.az;
    imu_or.gx =  imu.gy;
    imu_or.gy = -imu.gx;
    imu_or.gz =  imu.gz;

    return imu_or;
}

// ============================================================
// Gyro calibration
// ============================================================

bool calibrateGyroscope(
    MPU6050& mpu,
    AttitudeEstimator& estimator)
{
    std::cout
        << "Gyroscope calibration start."
        << std::endl;

    std::cout
        << "Keep the robot completely stationary."
        << std::endl;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    int valid_samples = 0;

    for (int i = 0;
         i < GYRO_CALIBRATION_SAMPLES;
         ++i)
    {
        MPU6050Raw raw;

        if (mpu.readRaw(raw))
        {
            IMUData imu = convertRawData(raw);

            sum_x += imu.gx;
            sum_y += imu.gy;
            sum_z += imu.gz;

            ++valid_samples;
        }

        usleep(
            static_cast<useconds_t>(
                1000000.0f / IMU_READ_RATE_HZ));
    }

    if (valid_samples == 0)
    {
        std::cerr
            << "Gyroscope calibration failed."
            << std::endl;

        return false;
    }

    const float offset_x =
        sum_x / valid_samples;

    const float offset_y =
        sum_y / valid_samples;

    const float offset_z =
        sum_z / valid_samples;

    estimator.setGyroOffset(
        offset_x,
        offset_y,
        offset_z);

    std::cout
        << "Gyroscope calibration completed."
        << std::endl;

    std::cout
        << "offset [rad/s] : "
        << offset_x << ", "
        << offset_y << ", "
        << offset_z
        << std::endl;

    return true;
}

// ============================================================
// main
// ============================================================

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "pp_champ_imu_hardware");

    ros::NodeHandle nh;

    ros::Publisher imu_pub =
        nh.advertise<sensor_msgs::Imu>(
            "/imu/data",
            10);

    MPU6050 mpu;

    if (!mpu.openDevice())
    {
        ROS_ERROR("Failed to open MPU6050.");
        return 1;
    }

    if (!mpu.initialize())
    {
        ROS_ERROR("Failed to initialize MPU6050.");
        return 1;
    }

    ROS_INFO("MPU6050 initialized.");

    AttitudeEstimator estimator;

    // --------------------------------------------------------
    // Gyroscope calibration
    // --------------------------------------------------------

    if (ENABLE_GYRO_CALIBRATION)
    {
        if (!calibrateGyroscope(
                mpu,
                estimator))
        {
            return 1;
        }
    }

    // --------------------------------------------------------
    // Timing
    // --------------------------------------------------------

    ros::Rate loop_rate(IMU_READ_RATE_HZ);

    ros::Time previous_time =
        ros::Time::now();

    float publish_accumulator = 0.0f;

    const float publish_period =
        1.0f / IMU_PUBLISH_RATE_HZ;

    // --------------------------------------------------------
    // Main loop
    // --------------------------------------------------------

    while (ros::ok())
    {
        MPU6050Raw raw;

        if (!mpu.readRaw(raw))
        {
            ROS_WARN_THROTTLE(
                1.0,
                "MPU6050 I2C read failed.");

            ros::spinOnce();
            loop_rate.sleep();

            continue;
        }

        IMUData imu =
            convertRawData(raw);

        // ----------------------------------------------------
        // Calculate actual dt
        // ----------------------------------------------------

        const ros::Time now =
            ros::Time::now();

        float dt =
            static_cast<float>(
                (now - previous_time).toSec());

        previous_time = now;

        // Reject abnormal dt
        //
        // This prevents a large integration jump
        // after scheduler delays etc.
        //
        if (dt <= 0.0f || dt > 0.1f)
        {
            dt = 1.0f / IMU_READ_RATE_HZ;
        }

        // ----------------------------------------------------
        // Attitude estimation
        // ----------------------------------------------------

        estimator.update(
            imu,
            dt);

        publish_accumulator += dt;

        // ----------------------------------------------------
        // Publish at 50 Hz
        // ----------------------------------------------------

        if (publish_accumulator >= publish_period)
        {
            publish_accumulator -= publish_period;

            sensor_msgs::Imu msg;

            msg.header.stamp = now;
            msg.header.frame_id = IMU_FRAME_ID;

            // ------------------------------------------------
            // Euler -> Quaternion
            // ------------------------------------------------

            tf2::Quaternion quaternion;

            quaternion.setRPY(
                estimator.roll(),
                estimator.pitch(),
                estimator.yaw());

            quaternion.normalize();

            msg.orientation.x = quaternion.x();
            msg.orientation.y = quaternion.y();
            msg.orientation.z = quaternion.z();
            msg.orientation.w = quaternion.w();

            // ------------------------------------------------
            // Angular velocity [rad/s]
            // ------------------------------------------------

            msg.angular_velocity.x = imu.gx;
            msg.angular_velocity.y = imu.gy;
            msg.angular_velocity.z = imu.gz;

            // ------------------------------------------------
            // Linear acceleration [m/s^2]
            // ------------------------------------------------

            msg.linear_acceleration.x = imu.ax;
            msg.linear_acceleration.y = imu.ay;
            msg.linear_acceleration.z = imu.az;

            // ------------------------------------------------
            // Covariance
            //
            // Initial provisional values.
            // Tune later after measurement.
            // ------------------------------------------------

            std::memset(
                msg.orientation_covariance.data(),
                0,
                sizeof(double) * 9);

            std::memset(
                msg.angular_velocity_covariance.data(),
                0,
                sizeof(double) * 9);

            std::memset(
                msg.linear_acceleration_covariance.data(),
                0,
                sizeof(double) * 9);

            msg.orientation_covariance[0] = 0.01;
            msg.orientation_covariance[4] = 0.01;

            // Yaw is less reliable because MPU6050
            // has no magnetometer.
            msg.orientation_covariance[8] =
                ENABLE_YAW_INTEGRATION
                    ? 1.0
                    : 999.0;

            msg.angular_velocity_covariance[0] = 0.01;
            msg.angular_velocity_covariance[4] = 0.01;
            msg.angular_velocity_covariance[8] = 0.01;

            msg.linear_acceleration_covariance[0] = 0.1;
            msg.linear_acceleration_covariance[4] = 0.1;
            msg.linear_acceleration_covariance[8] = 0.1;

            imu_pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
