#ifndef INCLUDE_AKFSFSIM_SIMULATION_H
#define INCLUDE_AKFSFSIM_SIMULATION_H

#include <chrono>  // For high-resolution timing
#include <memory>
#include <vector>

#include "kalmanfilter.h"
#include "display.h"
#include "car.h"
#include "beacons.h"
#include "sensors.h"

struct SensorsProfile
{
    double duration;

    bool gps_enabled;
    double gps_update_rate;
    double gps_position_noise_std;
    double gps_error_probability;
    double gps_denied_x;
    double gps_denied_y;
    double gps_denied_range;

    bool lidar_enabled;
    bool lidar_id_enabled;
    double lidar_update_rate;
    double lidar_range_noise_std;
    double lidar_theta_noise_std;
    double lidar_error_probability;

    bool imu_enabled;
    double imu_update_rate;
    double imu_error_probability;
    double accel_noise_std;
    double gyro_noise_std;
    double gyro_bias;

    bool compass_enabled;
    double compass_error_probability;
    double compass_update_rate;
    double compass_noise_std;
    double compass_bias;

    bool wheelspeed_enabled;
    double wheelspeed_error_probability;
    double wheelspeed_update_rate;
    double wheelspeed_noise_std;
    double wheelspeed_scaling_factor;


    SensorsProfile() :
        duration(0.0),
        gps_enabled(true), gps_update_rate(1.0), gps_position_noise_std(3), gps_error_probability(0.0),gps_denied_x(0.0),gps_denied_y(0.0),gps_denied_range(-1.0),
        lidar_enabled(true), lidar_id_enabled(true), lidar_update_rate(10.0),lidar_range_noise_std(3),lidar_theta_noise_std(0.02), lidar_error_probability(0.0),
        compass_enabled(true), compass_update_rate(5.0), compass_noise_std(0.05), compass_error_probability(0.0), compass_bias(0.0),
        wheelspeed_enabled(true), wheelspeed_update_rate(10.0), wheelspeed_noise_std(0.1), wheelspeed_error_probability(0.0), wheelspeed_scaling_factor(1.0),
        imu_enabled(true), imu_update_rate(10.0), accel_noise_std(0.05),gyro_noise_std(0.05), gyro_bias(0.0), imu_error_probability(0.0)
        {}
};


struct SimulationParams
{
    std::string profile_name;
    double time_step;
    double end_time;

    SensorsProfile cur_sensors_profile;
    
    double car_initial_x;
    double car_initial_y;
    double car_initial_psi;
    double car_initial_velocity;

    std::vector<std::shared_ptr<MotionCommandBase>> car_commands;
    std::queue<SensorsProfile> list_sensors_profile, list_sensors_profile_temp;

    SimulationParams():
        profile_name(""),
        time_step(0.1),end_time(120),
        car_initial_x(0.0),car_initial_y(0.0),car_initial_psi(0.0),car_initial_velocity(5.0)
    {}
};


class Simulation
{
    public:

        Simulation();
        void reset();

        std::string getSavePath();

        void save_plot(const std::string &filename);

        void save_metrics();

        void plot_trajectory(std::vector<Vector2> m_vehicle_position_history,
                             std::vector<Vector2> m_filter_position_history);

        void plot_error(std::vector<double> m_filter_error_position_history,
                        std::vector<double> m_filter_error_heading_history);

        void update_sensor_profile(SensorsProfile profile);

        void reset(SimulationParams sim_params);
        void update();
        void render(Display& disp);
        void increaseTimeMultiplier();
        void decreaseTimeMultiplier();
        void setTimeMultiplier(unsigned int multiplier);
        void increaseZoom();
        void decreaseZoom();
        void togglePauseSimulation();
        bool isPaused();
        bool isRunning();
        void selectFilter(unsigned int index);
        void toggleSensorGPS() {m_sim_parameters.cur_sensors_profile.gps_enabled = !m_sim_parameters.cur_sensors_profile.gps_enabled;}
        void toggleSensorLidar() {m_sim_parameters.cur_sensors_profile.lidar_enabled = !m_sim_parameters.cur_sensors_profile.lidar_enabled;}
        void toggleSensorCompass() {m_sim_parameters.cur_sensors_profile.compass_enabled = !m_sim_parameters.cur_sensors_profile.compass_enabled;}
        void toggleSensorWheelEncoder() {m_sim_parameters.cur_sensors_profile.wheelspeed_enabled = !m_sim_parameters.cur_sensors_profile.wheelspeed_enabled;}
        void toggleSensorIMU() {m_sim_parameters.cur_sensors_profile.imu_enabled = !m_sim_parameters.cur_sensors_profile.imu_enabled;}

    // private:

        SimulationParams m_sim_parameters;
        KalmanFilterBase *m_selected_filter;
        KalmanFilterLKF m_kalman_filter_lkf;
        KalmanFilterEKF m_kalman_filter_ekf;
        KalmanFilterUKF m_kalman_filter_ukf;
        OdometryFilter m_odometry_filter;
        Car m_car;
        BeaconMap m_beacons;
        GPSSensor m_gps_sensor;
        LidarSensor m_lidar_sensor;
        CompassSensor m_compass_sensor;
        WheelsSpeedSensor m_wheelspeed_sensor;
        IMUSensor m_imu_sensor;

        bool m_is_paused;
        bool m_is_running;
        int  m_time_multiplier;
        double m_view_size;

        double m_time;
        double m_time_till_sensor_profile_change;
        double m_time_till_gyro_measurement;
        double m_time_till_gps_measurement;
        double m_time_till_lidar_measurement;
        double m_time_till_compass_measurement;
        double m_time_till_wheelspeed_measurement;
        double m_time_till_imu_measurement;
        
        // CPU time tracking variables
        std::chrono::high_resolution_clock::time_point m_step_start_time;
        std::vector<double> m_cpu_times;
        double m_cpu_time_avg;

        double m_max_position_error;

        std::vector<GPSMeasurement> m_gps_measurement_history;
        std::vector<LidarMeasurement> m_lidar_measurement_history;

        std::vector<Vector2> m_vehicle_position_history;
        std::vector<Vector2> m_filter_position_history;

        std::vector<double> m_filter_error_position_history;
        std::vector<double> m_filter_error_heading_history;
        std::vector<double> m_filter_error_velocity_history;

};


#endif  // INCLUDE_AKFSFSIM_SIMULATION_H
