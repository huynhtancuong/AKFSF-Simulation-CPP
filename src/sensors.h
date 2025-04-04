#ifndef INCLUDE_AKFSFSIM_SENSORS_H
#define INCLUDE_AKFSFSIM_SENSORS_H

#include <random>
#include <vector>


struct GPSMeasurement{double x,y;};

struct LidarMeasurement{double range, psi;int id;};
struct IMUMeasurement{double accel, yaw_rate;};
struct WheelsSpeedMeasurement{double right_wheel_vel, left_wheel_vel, base_wheel_distance;};
struct CompassMeasurement{double theta;};

class BeaconMap;

class GPSSensor
{
    public:

        GPSSensor();
        void reset();
        void setGPSNoiseStd(double std);
        void setGPSErrorProb(double prob);
        void setGPSDeniedZone(double x, double y, double r);
        GPSMeasurement generateGPSMeasurement(double sensor_x, double sensor_y);

    private:

        std::mt19937 m_rand_gen;
        double m_noise_std;
        double m_error_prob;
        double m_gps_denied_x, m_gps_denied_y, m_gps_denied_range;

};


class LidarSensor
{
    public:

        LidarSensor();
        void reset();
        void setLidarNoiseStd(double range_std, double theta_std);
        void setLidarMaxRange(double range);
        void setLidarDAEnabled(bool id_enabled);
        std::vector<LidarMeasurement> generateLidarMeasurements(double sensor_x, double sensor_y, double sensor_yaw, const BeaconMap& map);

    private:

        std::mt19937 m_rand_gen;
        double m_range_noise_std;
        double m_theta_noise_std;
        double m_max_range;
        bool m_id_enabled;
};

class IMUSensor
{
    public:

        IMUSensor();
        void reset();
        void setAccelNoiseStd(double std);
        void setGyroNoiseStd(double std);
        IMUMeasurement generateIMUMeasurement(double sensor_accel, double sensor_yaw_rate);

        void setGyroBias(double gyro_bias);

    private:

        std::mt19937 m_rand_gen;
        double m_accel_noise_std, m_gyro_noise_std, m_gyro_bias;
};

class WheelsSpeedSensor
{
    public:

        WheelsSpeedSensor();
        void reset();
        void setOdometerNoiseStd(double std);
        WheelsSpeedMeasurement generateWheelsSpeedMeasurement(double sensor_left_wheel_vel, double sensor_right_wheel_vel, double base_wheel_distance);

    private:

        std::mt19937 m_rand_gen;
        double m_noise_std;
};

class CompassSensor
{
    public:

        CompassSensor();
        void reset();
        void setCompassNoiseStd(double std);
        CompassMeasurement generateCompassMeasurement(double sensor_heading);

    private:

        std::mt19937 m_rand_gen;
        double m_noise_std;
};

#endif  // INCLUDE_AKFSFSIM_SENSORS_H