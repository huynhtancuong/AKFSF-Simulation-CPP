
#include "simulation.h"
#include "utils.h"
#include <cmath>
#include <filesystem>
#include <matplot/matplot.h>

Simulation::Simulation():
    m_sim_parameters(SimulationParams()),
    m_odometry_filter(OdometryFilter("Odometry")),
    m_kalman_filter_lkf(KalmanFilterLKF("Linear Kalman Filter")),
    m_kalman_filter_ekf(KalmanFilterEKF("Extended Kalman Filter")),
    m_kalman_filter_ukf(KalmanFilterUKF("Unscented Kalman Filter")),
    m_selected_filter(&m_kalman_filter_ekf),
    m_is_paused(false),
    m_is_running(false),
    m_time_multiplier(5),
    m_view_size(100),
    m_time(0.0),
    m_time_till_gyro_measurement(0.0),
    m_time_till_gps_measurement(0.0),
    m_time_till_lidar_measurement(0.0),
    m_time_till_wheelspeed_measurement(0.0),
    m_time_till_imu_measurement(0.0) {
}

void Simulation::reset()
{
    // Reset Simulation
    m_time = 0.0;
    m_time_till_gyro_measurement = 0.0;
    m_time_till_gps_measurement = 0.0;
    m_time_till_lidar_measurement= 0.0;

    m_is_running = true;
    m_is_paused = false;
    
    // Reset CPU time tracking variables
    m_cpu_times.clear();
    m_cpu_time_avg = 0.0;
    
    m_kalman_filter_lkf.reset();
    m_kalman_filter_ekf.reset();
    m_kalman_filter_ukf.reset();
    m_odometry_filter.reset();

    m_gps_sensor.reset();
    m_gps_sensor.setGPSNoiseStd(m_sim_parameters.gps_position_noise_std);
    m_gps_sensor.setGPSErrorProb(m_sim_parameters.gps_error_probability);
    m_gps_sensor.setGPSDeniedZone(m_sim_parameters.gps_denied_x, m_sim_parameters.gps_denied_y, m_sim_parameters.gps_denied_range);

    m_imu_sensor.reset();
    m_imu_sensor.setErrorProb(m_sim_parameters.imu_error_probability);
    m_imu_sensor.setGyroNoiseStd(m_sim_parameters.gyro_noise_std);
    m_imu_sensor.setGyroBias(m_sim_parameters.gyro_bias);
    m_imu_sensor.setAccelNoiseStd(m_sim_parameters.accel_noise_std);

    m_wheelspeed_sensor.reset();
    m_wheelspeed_sensor.setErrorProb(m_sim_parameters.wheelspeed_error_probability);
    m_wheelspeed_sensor.setOdometerNoiseStd(m_sim_parameters.wheelspeed_noise_std);
    m_wheelspeed_sensor.setScalingFactor(m_sim_parameters.wheelspeed_scaling_factor);

    m_compass_sensor.reset();
    m_compass_sensor.setErrorProb(m_sim_parameters.compass_error_probability);
    m_compass_sensor.setCompassNoiseStd(m_sim_parameters.compass_noise_std);
    m_compass_sensor.setBias(m_sim_parameters.compass_bias);

    m_lidar_sensor.reset();
    m_lidar_sensor.setLidarErrorProb(m_sim_parameters.lidar_error_probability);
    m_lidar_sensor.setLidarNoiseStd(m_sim_parameters.lidar_range_noise_std, m_sim_parameters.lidar_theta_noise_std);
    m_lidar_sensor.setLidarDAEnabled(m_sim_parameters.lidar_id_enabled);

    m_car.reset(m_sim_parameters.car_initial_x, m_sim_parameters.car_initial_y,m_sim_parameters.car_initial_psi, m_sim_parameters.car_initial_velocity);
    
    for(auto& cmd : m_sim_parameters.car_commands){m_car.addVehicleCommand(cmd.get());}

    // Plotting Variables
    m_gps_measurement_history.clear();
    m_lidar_measurement_history.clear();
    m_vehicle_position_history.clear();
    m_filter_position_history.clear();

    // Stats Variables
    m_filter_error_position_history.clear();
    m_filter_error_heading_history.clear();
    m_filter_error_velocity_history.clear();

    std::cout << "Simulation: Reset" << std::endl;
}

std::string Simulation::getSavePath()
{
    std::string path = "/home/huynh/repos/kalman-filter-course-udemy/simulation/images/"
                        + m_sim_parameters.profile_name + "/" + m_selected_filter->getName() + "/";
    // Replace space with underscore
    path.replace(path.find(" - "), 3, "_");
    std::replace(path.begin(), path.end(), ' ', '_');
    return path;
}

void Simulation::save_plot(const std::string& filename)
{
    using namespace matplot;
    std::string path = getSavePath() +  filename;
    std::cout << "Saving plot to: " << path << std::endl;
    // Create folder if it doesn't exist
    std::filesystem::path dir(path);
    if (!std::filesystem::exists(dir.parent_path()))
    {
        std::filesystem::create_directories(dir.parent_path());
    }
    // Save the current figure to a file
    save(path);
}

void Simulation::save_metrics()
{
    // Save the metrics to a CSV file
    std::string path = getSavePath() + "metrics.csv";
    // Write to file
    double position_RMSE = calculateRMSE(m_filter_error_position_history);
    double heading_RMSE = calculateRMSE(m_filter_error_heading_history);
    double cpu_time_avg = m_cpu_time_avg;

    // Open file
    std::ofstream file(path);
    if (file.is_open())
    {
        // Write header
        file << "Position RMSE,Heading RMSE,CPU Time Avg\n";
        // Write data
        file << position_RMSE << "," << heading_RMSE << "," << cpu_time_avg << "\n";
        file.close();
        std::cout << "Metrics saved to: " << path << std::endl;
    }
    else
    {
        std::cerr << "Error: Unable to open file " << path << std::endl;
    }
}

void Simulation::plot_trajectory(std::vector<Vector2> m_vehicle_position_history, std::vector<Vector2> m_filter_position_history)
{
    using namespace matplot;

    // Get the x and y coordinates of the vehicle position history
    std::vector<double> true_x, true_y;
    for (const auto& pos : m_vehicle_position_history)
    {
        true_x.push_back(pos.x);
        true_y.push_back(pos.y);
    }

    // Get the x and y coordinates of the filter position history
    std::vector<double> filter_x, filter_y;
    for (const auto& pos : m_filter_position_history)
    {
        filter_x.push_back(pos.x);
        filter_y.push_back(pos.y);
    }

    auto fig = figure(true);
    fig->size(800, 800);
    plot(true_x, true_y)->color("green").line_width(2).display_name("Ground Truth");
    hold(on);
    plot(filter_x, filter_y)->color("red").line_width(2).display_name("Estimation");
    grid(on);
    xlabel("x (m)");
    ylabel("y (m)");
    legend();
    title("Vehicle Trajectory");
    save_plot("trajectory.png");
    // show();
}

void Simulation::plot_error(std::vector<double> m_filter_error_position_history, std::vector<double> m_filter_error_heading_history, std::vector<double> m_filter_error_velocity_history)
{
    using namespace matplot;

    std::vector<double> time;
    for (unsigned i = 0; i < m_filter_error_position_history.size(); ++i)
    {
        time.push_back(i/10.0);
    }

    auto fig = figure(true);
    // fig->size(800, 1000);
    // subplot(2, 1, 0);
    hold(on);
    plot(time, m_filter_error_position_history)->color("red").line_width(2);
    grid(on);
    xlabel("Time (s)");
    ylabel("Meters");
    title("Position Error");
    save_plot("position_error.png");
    // show();

    auto fig2 = figure(true);
    // subplot(2, 1, 1);
    plot(time, m_filter_error_heading_history)->color("red").line_width(2);
    hold(on);
    plot(time, transform(m_filter_error_heading_history, [](double x) { return 0; }))->color("black").line_width(1).display_name("Zero Line");
    title("Heading Error");
    xlabel("Time (s)");
    ylabel("Radians");
    grid(on);
    save_plot("heading_error.png");

    // subplot(3, 1, 2);
    // plot(time, m_filter_error_velocity_history)->color("red").line_width(1);
    // hold(on);
    // plot(time, transform(m_filter_error_heading_history, [](double x) { return 0; }))->color("black").line_width(1).display_name("Zero Line");
    // title("Velocity Error");
    // xlabel("Time (s)");
    // ylabel("Meters/Second");
    // grid(on);

    // show();
}

void Simulation::update()
{
    if (m_is_running && !m_is_paused)
    {
        // Start timing this iteration
        m_step_start_time = std::chrono::high_resolution_clock::now();
        
        // Time Multiplier
        for (unsigned i = 0; i < m_time_multiplier; ++i)
        {
            // Check for End Time
            if(m_time >= m_sim_parameters.end_time)
            {
                m_is_running = false;
                std::cout << "Simulation: Reached End of Simulation Time (" << m_time << ")" << std::endl;
                plot_trajectory(m_vehicle_position_history, m_filter_position_history);
                plot_error(m_filter_error_position_history, m_filter_error_heading_history, m_filter_error_velocity_history);
                save_metrics();
                return;
            }

            // Update Motion
            m_car.update(m_time, m_sim_parameters.time_step);
            m_vehicle_position_history.push_back(Vector2(m_car.getVehicleState().x,m_car.getVehicleState().y));

            // Update with IMU
            if (m_sim_parameters.imu_enabled)
            {
                if (m_time_till_imu_measurement <= 0)
                {
                    IMUMeasurement meas = m_imu_sensor.generateIMUMeasurement(m_car.getVehicleState().accel, m_car.getVehicleState().yaw_rate);
                    m_selected_filter->predictionStep(meas, m_sim_parameters.time_step);
                    m_time_till_imu_measurement += 1.0/m_sim_parameters.imu_update_rate;
                }
                m_time_till_imu_measurement -= m_sim_parameters.time_step;
            }
            else
            {
                m_selected_filter->predictionStep(m_sim_parameters.time_step);
            }


            // Wheel Encoder Measurement
            if (m_sim_parameters.wheelspeed_enabled)
            {
                if (m_time_till_wheelspeed_measurement <= 0)
                {
                    WheelsSpeedMeasurement meas = m_wheelspeed_sensor.generateWheelsSpeedMeasurement(m_car.getVehicleState().right_wheel_velocity,
                                                                                                 m_car.getVehicleState().left_wheel_velocity,
                                                                                                 m_car.getVehicleState().base_wheel_distance);
                    m_selected_filter->handleWheelsSpeedMeasurement(meas);
                    m_time_till_wheelspeed_measurement += 1.0/m_sim_parameters.wheelspeed_update_rate;
                }
                m_time_till_wheelspeed_measurement -= m_sim_parameters.time_step;
            }

            // Compass Measurement
            if (m_sim_parameters.compass_enabled)
            {
                if (m_time_till_compass_measurement <= 0)
                {
                    CompassMeasurement meas = m_compass_sensor.generateCompassMeasurement(m_car.getVehicleState().theta);
                    m_selected_filter->handleCompassMeasurement(meas);
                    m_time_till_compass_measurement += 1.0/m_sim_parameters.compass_update_rate;
                }
                m_time_till_compass_measurement -= m_sim_parameters.time_step;
            }

            // GPS Measurement
            if (m_sim_parameters.gps_enabled)
            {
                if (m_time_till_gps_measurement <= 0)
                {
                    GPSMeasurement gps_meas = m_gps_sensor.generateGPSMeasurement(m_car.getVehicleState().x,m_car.getVehicleState().y);
                    m_selected_filter->handleGPSMeasurement(gps_meas);
                    m_gps_measurement_history.push_back(gps_meas);
                    m_time_till_gps_measurement += 1.0/m_sim_parameters.gps_update_rate;
                }
                m_time_till_gps_measurement -= m_sim_parameters.time_step;
            }

            // Lidar Measurement
            if (m_sim_parameters.lidar_enabled)
            {
                if (m_time_till_lidar_measurement <= 0)
                {
                    std::vector<LidarMeasurement> lidar_measurements = m_lidar_sensor.generateLidarMeasurements(m_car.getVehicleState().x,m_car.getVehicleState().y, m_car.getVehicleState().theta, m_beacons);
                    m_selected_filter->handleLidarMeasurements(lidar_measurements, m_beacons);
                    m_lidar_measurement_history = lidar_measurements;
                    m_time_till_lidar_measurement += 1.0/m_sim_parameters.lidar_update_rate;
                }
                m_time_till_lidar_measurement -= m_sim_parameters.time_step;
            }


            // Save Filter History and Calculate Stats
            if (m_selected_filter->isInitialised())
            {
                VehicleState vehicle_state = m_car.getVehicleState();
                VehicleState filter_state = m_selected_filter->getVehicleState();
                m_filter_position_history.push_back(Vector2(filter_state.x, filter_state.y));
                
                // Calculate combined position error (Euclidean distance)
                double pos_error_x = filter_state.x - vehicle_state.x;
                double pos_error_y = filter_state.y - vehicle_state.y;
                double pos_error = std::sqrt(pos_error_x*pos_error_x + pos_error_y*pos_error_y);
                m_filter_error_position_history.push_back(pos_error);
                
                m_filter_error_heading_history.push_back(wrapAngle(filter_state.theta - vehicle_state.theta));
                m_filter_error_velocity_history.push_back(filter_state.V - vehicle_state.V);
            }

            // Update Time
            m_time += m_sim_parameters.time_step;
        }
                    
        // Calculate CPU time for this iteration
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end_time - m_step_start_time;

        // Store CPU time in milliseconds
        m_cpu_times.push_back(elapsed.count());

        // Limit history to last 100 values to avoid excessive memory usage
        if (m_cpu_times.size() > 100)
            m_cpu_times.erase(m_cpu_times.begin());

        // Calculate average CPU time
        if (!m_cpu_times.empty())
            m_cpu_time_avg = std::accumulate(m_cpu_times.begin(), m_cpu_times.end(), 0.0) / m_cpu_times.size();
    }
}
        
void Simulation::render(Display& disp)
{
    std::vector<Vector2> marker_lines1 = {{0.5,0.5},{-0.5,-0.5}};
    std::vector<Vector2> marker_lines2 = {{0.5,-0.5},{-0.5,0.5}};

    disp.setView(m_view_size * disp.getScreenAspectRatio(),m_view_size, m_car.getVehicleState().x, m_car.getVehicleState().y);

    m_car.render(disp);
    m_beacons.render(disp);

    disp.setDrawColour(0,150,0);
    disp.drawLines(m_vehicle_position_history, 1);

    disp.setDrawColour(150,0,0);
    disp.drawLines(m_filter_position_history, 1);

    if (m_selected_filter->isInitialised())
    {
        VehicleState filter_state = m_selected_filter->getVehicleState();
        Eigen::Matrix2d cov = m_selected_filter->getVehicleStatePositionCovariance();

        double x = filter_state.x;
        double y = filter_state.y;
        double sigma_xx = cov(0,0);
        double sigma_yy = cov(1,1);
        double sigma_xy = cov(0,1);

        std::vector<Vector2> marker_lines1_world = offsetPoints(marker_lines1, Vector2(x,y));
        std::vector<Vector2> marker_lines2_world = offsetPoints(marker_lines2, Vector2(x,y));
        disp.setDrawColour(255,0,0);
        disp.drawLines(marker_lines1_world, 2);
        disp.drawLines(marker_lines2_world, 2);

        std::vector<Vector2> cov_world = generateEllipse(x,y,sigma_xx,sigma_yy,sigma_xy);
        disp.setDrawColour(255,0,0);
        disp.drawLines(cov_world, 2);

    }

    // Render GPS Measurements
    std::vector<std::vector<Vector2>> m_gps_marker = {{{0.5,0.5},{-0.5,-0.5}}, {{0.5,-0.5},{-0.5,0.5}}};
    disp.setDrawColour(255,255,255);
    for(const auto& meas : m_gps_measurement_history){disp.drawLines(offsetPoints(m_gps_marker, Vector2(meas.x,meas.y)));}

    // Render GPS Denied Zone
    if(m_sim_parameters.gps_denied_range > 0)
    {
        std::vector<Vector2> zone_lines = generateCircle(m_sim_parameters.gps_denied_x, m_sim_parameters.gps_denied_y, m_sim_parameters.gps_denied_range);
        disp.setDrawColour(255,150,0);
        disp.drawLines(zone_lines);
    }

    // Render Lidar Measurements
    if (m_sim_parameters.lidar_enabled)
    {
        for(const auto& meas : m_lidar_measurement_history)
        {
            double x0 = m_car.getVehicleState().x;
            double y0 = m_car.getVehicleState().y;
            double delta_x = meas.range * cos(meas.psi + m_car.getVehicleState().theta);
            double delta_y = meas.range * sin(meas.psi + m_car.getVehicleState().theta);
            disp.setDrawColour(201,201,0);
            disp.drawLine(Vector2(x0,y0), Vector2(x0 + delta_x,y0 + delta_y));
        }
    }

    int x_offset, y_offset; 
    int stride = 20;
    // Simulation Status / Parameters
    x_offset = 10;
    y_offset = 30;
    int line_num = -1;
    std::string profile_string =        string_format("Profile: %s", m_sim_parameters.profile_name.c_str());
    std::string filter_type_string =    "Current Filter Type: " + m_selected_filter->getName();
    std::string time_string =           string_format("Time: %0.2f (x%d)",m_time,m_time_multiplier);
    std::string sensors_string =        "Sensors: ";
    std::string gps_string =            string_format("                      GPS [g]: %s (%0.1f Hz)", (m_sim_parameters.gps_enabled ? "ON" : "OFF"), m_sim_parameters.gps_update_rate);
    std::string lidar_string =          string_format("                  LIDAR [d]: %s (%0.1f Hz) ", (m_sim_parameters.lidar_enabled ? "ON" : "OFF"), m_sim_parameters.lidar_update_rate);
    std::string imu_string =            string_format("                        IMU [i]: %s (%0.1f Hz)", (m_sim_parameters.imu_enabled ? "ON" : "OFF"), m_sim_parameters.imu_update_rate);
    std::string compass_string =        string_format("            Compass [c]: %s (%0.1f Hz)", (m_sim_parameters.compass_enabled ? "ON" : "OFF"), m_sim_parameters.compass_update_rate);
    std::string wheelencoder_string =   string_format("Wheel Encoder [w]: %s (%0.1f Hz)", (m_sim_parameters.wheelspeed_enabled ? "ON" : "OFF"), m_sim_parameters.wheelspeed_update_rate);
    disp.drawText_MainFont(profile_string,Vector2(x_offset,y_offset+stride*line_num++),1.0,{255,255,255});
    disp.drawText_MainFont(filter_type_string,Vector2(x_offset,y_offset+stride*line_num++),1.0,{255,255,0});
    disp.drawText_MainFont(time_string,Vector2(x_offset,y_offset+stride*line_num++),1.0,{255,255,255});
    disp.drawText_MainFont(sensors_string,Vector2(x_offset,y_offset+stride*line_num++),1.0,{255,255,255});
    disp.drawText_Color_Condition(gps_string,Vector2(x_offset,y_offset+stride*line_num++), m_sim_parameters.gps_enabled);
    disp.drawText_Color_Condition(lidar_string,Vector2(x_offset,y_offset+stride*line_num++), m_sim_parameters.lidar_enabled);
    disp.drawText_Color_Condition(imu_string,Vector2(x_offset,y_offset+stride*line_num++), m_sim_parameters.imu_enabled);
    disp.drawText_Color_Condition(compass_string,Vector2(x_offset,y_offset+stride*line_num++), m_sim_parameters.compass_enabled);
    disp.drawText_Color_Condition(wheelencoder_string,Vector2(x_offset,y_offset+stride*line_num++), m_sim_parameters.wheelspeed_enabled);
    if (m_is_paused){disp.drawText_MainFont("PAUSED",Vector2(x_offset,y_offset+stride*line_num++),1.0,{255,0,0});}
    if (!m_is_running){disp.drawText_MainFont("FINISHED",Vector2(x_offset,y_offset+stride*line_num++),1.0,{255,0,0});}

    // Vehicle State
    x_offset = 800;
    y_offset = 10;
    std::string velocity_string =       string_format("    Velocity: %0.2f m/s",m_car.getVehicleState().V);
    std::string yaw_string =            string_format("   Heading: %0.2f deg",m_car.getVehicleState().theta * 180.0/M_PI);
    std::string yaw_rate_string =       string_format(" Yaw Rate: %0.2f deg/s",m_car.getVehicleState().yaw_rate * 180.0/M_PI);
    std::string xpos =                  string_format("X Position: %0.2f m",m_car.getVehicleState().x);
    std::string ypos =                  string_format("Y Position: %0.2f m",m_car.getVehicleState().y);
    disp.drawText_MainFont("Vehicle State",Vector2(x_offset-5,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont(velocity_string,Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont(yaw_string,Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    disp.drawText_MainFont(xpos,Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
    disp.drawText_MainFont(ypos,Vector2(x_offset,y_offset+stride*4),1.0,{255,255,255});
    disp.drawText_MainFont(yaw_rate_string,Vector2(x_offset,y_offset+stride*5),1.0,{255,255,255});

    std::string kf_velocity_string = string_format("    Velocity: %0.2f m/s",m_selected_filter->getVehicleState().V);
    std::string kf_yaw_string = string_format("   Heading: %0.2f deg",m_selected_filter->getVehicleState().theta * 180.0/M_PI);
    std::string kf_xpos = string_format("X Position: %0.2f m",m_selected_filter->getVehicleState().x);
    std::string kf_ypos = string_format("Y Position: %0.2f m",m_selected_filter->getVehicleState().y);
    std::string kf_yaw_rate = string_format("   Yaw Rate: %0.2f deg/s",m_selected_filter->getVehicleState().yaw_rate * 180.0/M_PI);
    disp.drawText_MainFont("Filter State",Vector2(x_offset,y_offset+stride*8),1.0,{255,255,255});
    disp.drawText_MainFont(kf_velocity_string,Vector2(x_offset,y_offset+stride*9),1.0,{255,255,255});
    disp.drawText_MainFont(kf_yaw_string,Vector2(x_offset,y_offset+stride*10),1.0,{255,255,255});
    disp.drawText_MainFont(kf_xpos,Vector2(x_offset,y_offset+stride*11),1.0,{255,255,255});
    disp.drawText_MainFont(kf_ypos,Vector2(x_offset,y_offset+stride*12),1.0,{255,255,255});
    disp.drawText_MainFont(kf_yaw_rate,Vector2(x_offset,y_offset+stride*13),1.0,{255,255,255});

    // Keyboard Input
    x_offset = 10;
    y_offset = 650;
    disp.drawText_MainFont("Select Filter Type: (O/L/E/U)",Vector2(x_offset,y_offset+stride*-1),1.0,{255,255,255});
    disp.drawText_MainFont("Reset Key: r",Vector2(x_offset,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont("Pause Key: [space bar]",Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont("Speed Multiplier (+/-) Key: [ / ] ",Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    disp.drawText_MainFont("Zoom (+/-) Key: PgUp / PgDn",Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
    disp.drawText_MainFont("Motion Profile Key: 1 - 9,0",Vector2(x_offset,y_offset+stride*4),1.0,{255,255,255});


    // Filter Error State
    x_offset = 750;
    y_offset = 650;
    std::string pos_error_string = string_format("Position RMSE: %0.2f m",calculateRMSE(m_filter_error_position_history));
    std::string heading_error_string = string_format("   Heading RMSE: %0.2f rad",calculateRMSE(m_filter_error_heading_history));
    std::string velocity_error_string = string_format("    Velocity RMSE: %0.2f m/s",calculateRMSE(m_filter_error_velocity_history));
    disp.drawText_MainFont(pos_error_string,Vector2(x_offset,y_offset+stride*0),1.0,{255,255,255});
    disp.drawText_MainFont(heading_error_string,Vector2(x_offset,y_offset+stride*1),1.0,{255,255,255});
    disp.drawText_MainFont(velocity_error_string,Vector2(x_offset,y_offset+stride*2),1.0,{255,255,255});
    
    // CPU Time metrics
    std::string current_cpu_time = string_format("Current CPU Time: %0.2f ms", m_cpu_times.empty() ? 0.0 : m_cpu_times.back());
    std::string avg_cpu_time = string_format("Avg CPU Time: %0.2f ms", m_cpu_time_avg);
    disp.drawText_MainFont(current_cpu_time,Vector2(x_offset,y_offset+stride*3),1.0,{255,255,255});
    disp.drawText_MainFont(avg_cpu_time,Vector2(x_offset,y_offset+stride*4),1.0,{255,255,255});
}
   
void Simulation::reset(SimulationParams sim_params){m_sim_parameters = sim_params; reset();}
void Simulation::increaseTimeMultiplier()
{
    m_time_multiplier++;
    std::cout << "Simulation: Time Multiplier Increased (x" << m_time_multiplier << ")" << std::endl;
}
void Simulation::decreaseTimeMultiplier()
{
    if (m_time_multiplier > 1)
    {
        m_time_multiplier--;
        std::cout << "Simulation: Time Multiplier Decreased (x" << m_time_multiplier << ")" << std::endl;
    }
}
void Simulation::setTimeMultiplier(unsigned int multiplier){m_time_multiplier = static_cast<int>(multiplier);}
void Simulation::increaseZoom()
{
    if (m_view_size > 25){m_view_size -= 25;}
    std::cout << "Simulation: Zoom Increased (" << m_view_size << "m)" << std::endl;
}
void Simulation::decreaseZoom()
{
    if (m_view_size < 400){m_view_size += 25;}
    std::cout << "Simulation: Zoom Decreased (" << m_view_size << "m)" << std::endl;
}
void Simulation::togglePauseSimulation()
{
    m_is_paused = !m_is_paused;
    std::cout << "Simulation: Paused (" << (m_is_paused?"True":"False") << ")" << std::endl;
}
bool Simulation::isPaused(){return m_is_paused;}
bool Simulation::isRunning(){return m_is_running;}
void Simulation::selectFilter(unsigned int index)
{
    switch (index) {
        case 0:
            m_selected_filter = &m_kalman_filter_lkf;
            std::cout << "Simulation: Selected LKF" << std::endl;
            m_sim_parameters.compass_enabled = false;
            m_sim_parameters.gps_enabled = true;
            m_sim_parameters.imu_enabled = false;
            m_sim_parameters.lidar_enabled = false;
            m_sim_parameters.wheelspeed_enabled = false;
            break;
        case 1:
            m_selected_filter = &m_kalman_filter_ekf;
            std::cout << "Simulation: Selected EKF" << std::endl;
            // m_sim_parameters.compass_enabled = true;
            // m_sim_parameters.gps_enabled = true;
            // m_sim_parameters.imu_enabled = true;
            // m_sim_parameters.lidar_enabled = false;
            // m_sim_parameters.wheelspeed_enabled = true;
            break;
        case 2:
            m_selected_filter = &m_kalman_filter_ukf;
            std::cout << "Simulation: Selected UKF" << std::endl;
            // m_sim_parameters.compass_enabled = true;
            // m_sim_parameters.gps_enabled = true;
            // m_sim_parameters.imu_enabled = true;
            // m_sim_parameters.lidar_enabled = false;
            // m_sim_parameters.wheelspeed_enabled = true;
            break;
        case 3:
            m_selected_filter = &m_odometry_filter;
            std::cout << "Simulation: Selected Odometry Filter" << std::endl;
            m_sim_parameters.compass_enabled = false;
            m_sim_parameters.gps_enabled = true;
            m_sim_parameters.imu_enabled = false;
            m_sim_parameters.lidar_enabled = false;
            m_sim_parameters.wheelspeed_enabled = true;
            break;
        default: ;
    }
    reset();
}
