// System Includes
#include <string>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "simulation.h"
#include "car.h"
#include "display.h"
#include <cmath>
#include <filesystem>
#include <matplot/matplot.h>

// Screen dimension constants
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;
const double GRID_SIZE = 500;
const double GRID_SPACEING = 25;

// Function Prototypes
SimulationParams loadSimulation1Parameters();
SimulationParams loadSimulation2Parameters();
SimulationParams loadSimulation3Parameters();
SimulationParams loadSimulation4Parameters();
SimulationParams loadSimulation5Parameters();
SimulationParams loadSimulation6Parameters();
SimulationParams loadSimulation7Parameters();
SimulationParams loadSimulation8Parameters();
SimulationParams loadSimulation9Parameters();
SimulationParams loadSimulation0Parameters();

struct FilterSimulationData {
    std::string filter_name;
    std::vector<Vector2> vehicle_path;
    std::vector<Vector2> filter_path;
    double position_rmse, max_position_error;
    double heading_rmse;
    double avg_cpu_time;
    std::vector<double> position_error;
    std::vector<double> heading_error;
};

void saveProfileSimData(std::vector <FilterSimulationData> filter_simulation_data, std::string profile_name)
{
    using namespace matplot;
    std::string path = "/home/huynh/repos/kalman-filter-course-udemy/simulation/images/"
                        + profile_name + "/";
    path.replace(path.find(" - "), 3, "_");
    std::replace(path.begin(), path.end(), ' ', '_');

    // Save trajectory plot
    std::string trajectory_path = path + "trajectory.png";
    std::vector<double> true_x, true_y;
    for (const auto& pos : filter_simulation_data[0].vehicle_path)
    {
        true_x.push_back(pos.x);
        true_y.push_back(pos.y);
    }
    auto fig = figure(true);
    fig->size(800, 800);
    plot(true_x, true_y)->color("green").line_width(1).display_name("Ground Truth");
    hold(on);
    for (const auto& filter_data : filter_simulation_data)
    {
        std::vector<double> filter_x, filter_y;
        for (const auto& pos : filter_data.filter_path)
        {
            filter_x.push_back(pos.x);
            filter_y.push_back(pos.y);
        }
        plot(filter_x, filter_y)->line_width(1).display_name(filter_data.filter_name);
    }
    grid(on);
    xlabel("x (m)");
    ylabel("y (m)");
    legend();
    title("Vehicle Trajectory");
    save(trajectory_path);

    // Save position error plot
    std::vector<double> time;
    for (unsigned i = 0; i < filter_simulation_data[0].position_error.size(); ++i)
    {
        time.push_back(i/10.0);
    }
    auto fig2 = figure(true);
    hold(on);
    for (const auto& filter_data : filter_simulation_data)
    {
        plot(time, filter_data.position_error)->line_width(1).display_name(filter_data.filter_name);
    }
    grid(on);
    xlabel("Time (s)");
    ylabel("Meters");
    title("Position Error");
    legend();
    save(path + "position_error.png");

    // Save heading error plot
    auto fig3 = figure(true);
    hold(on);
    for (const auto& filter_data : filter_simulation_data)
    {
        plot(time, filter_data.heading_error)->line_width(1).display_name(filter_data.filter_name);
    }
    grid(on);
    xlabel("Time (s)");
    ylabel("Radians");
    title("Heading Error");
    legend();
    save(path + "heading_error.png");
}


void saveMetricsData(std::vector <FilterSimulationData> filter_simulation_data, std::string profile_name)
{
    std::string path = "/home/huynh/repos/kalman-filter-course-udemy/simulation/images/"
                        + profile_name + "/";
    path.replace(path.find(" - "), 3, "_");
    std::replace(path.begin(), path.end(), ' ', '_');

    // Save metrics data to CSV file
    std::string metrics_path = path + "metrics.csv";
    std::ofstream metrics_file(metrics_path);
    if (metrics_file.is_open())
    {
        metrics_file << "Filter Name,Position RMSE,Max Position Error,Heading RMSE,Mean CPU Time\n";
        for (const auto& filter_data : filter_simulation_data)
        {
            metrics_file << filter_data.filter_name << ","
                         << filter_data.position_rmse << ","
                         << filter_data.max_position_error << ","
                         << filter_data.heading_rmse << ","
                         << filter_data.avg_cpu_time << "\n";
        }
        metrics_file.close();
    }
    else
    {
        std::cerr << "Unable to open file: " << metrics_path << std::endl;
    }
}

// Main Loop
int main( int argc, char* args[] )
{
    Display mDisplay;
    Simulation mSimulation;
    
    // Start Graphics
    if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
    {
        std::cout << "SDL could not initialize! SDL_Error: " <<  SDL_GetError() << std::endl;
        return -1;
    }
    if( TTF_Init() == -1 )
    {
        std::cout << "SDL_ttf could not initialize! SDL_ttf Error: " << TTF_GetError() << std::endl;
        return -1;
    }

    // Create Display
    if (!mDisplay.createRenderer("AKFSF Simulations", SCREEN_WIDTH, SCREEN_HEIGHT)){return false;}

    // Main Simulation Loop
    // mSimulation.reset(loadSimulation1Parameters());

    mSimulation.setTimeMultiplier(100);

    bool mRunning = true;


    std::vector<SimulationParams> sim_params;
    sim_params.push_back(loadSimulation1Parameters());
    sim_params.push_back(loadSimulation2Parameters());
    sim_params.push_back(loadSimulation3Parameters());
    sim_params.push_back(loadSimulation4Parameters());
    sim_params.push_back(loadSimulation5Parameters());
    sim_params.push_back(loadSimulation6Parameters());
    sim_params.push_back(loadSimulation7Parameters());
    sim_params.push_back(loadSimulation8Parameters());

    std::vector<FilterSimulationData> filter_simulation_data(4);

    // Automatic switch sim profile and filter
    for (const auto& sim_param : sim_params)
    {
        for (int filter_type = 0; filter_type < 4; ++filter_type)
        {
            mSimulation.reset(sim_param);
            mSimulation.selectFilter(filter_type);
            std::cout << "Simulation: Running Simulation " << sim_param.profile_name << " with Filter " << filter_type << std::endl;

            // Update Simulation
            while (mSimulation.isRunning() && mRunning)
            {
                mSimulation.update();

                // Update Display
                mDisplay.clearScreen();

                // Draw Background Grid
                mDisplay.setDrawColour(101,101,101);
                for (int x = -GRID_SIZE; x <= GRID_SIZE; x+=GRID_SPACEING){mDisplay.drawLine(Vector2(x,-GRID_SIZE),Vector2(x,GRID_SIZE));}
                for (int y = -GRID_SIZE; y <= GRID_SIZE; y+=GRID_SPACEING){mDisplay.drawLine(Vector2(-GRID_SIZE,y),Vector2(GRID_SIZE,y));}

                // Draw Simulation
                mSimulation.render(mDisplay);

                mDisplay.showScreen();

                // Handle Events
                SDL_Event event;
                while( SDL_PollEvent( &event ) != 0 )
                {
                    if( event.type == SDL_QUIT ){mRunning = false;}
                    else if (event.type == SDL_KEYDOWN)
                    {
                        switch( event.key.keysym.sym )
                        {
                            case SDLK_SPACE: mSimulation.togglePauseSimulation(); break;
                            case SDLK_ESCAPE:mRunning = false; break;
                            case SDLK_PAGEUP: mSimulation.increaseZoom(); break;
                            case SDLK_PAGEDOWN: mSimulation.decreaseZoom(); break;
                            case SDLK_RIGHTBRACKET: mSimulation.increaseTimeMultiplier(); break;
                            case SDLK_LEFTBRACKET: mSimulation.decreaseTimeMultiplier(); break;
                            case SDLK_r: mSimulation.reset(); break;
                            case SDLK_1: mSimulation.reset(loadSimulation1Parameters()); break;
                            case SDLK_2: mSimulation.reset(loadSimulation2Parameters()); break;
                            case SDLK_3: mSimulation.reset(loadSimulation3Parameters()); break;
                            case SDLK_4: mSimulation.reset(loadSimulation4Parameters()); break;
                            case SDLK_5: mSimulation.reset(loadSimulation5Parameters()); break;
                            case SDLK_6: mSimulation.reset(loadSimulation6Parameters()); break;
                            case SDLK_7: mSimulation.reset(loadSimulation7Parameters()); break;
                            // case SDLK_8: mSimulation.reset(loadSimulation8Parameters()); break;
                            // case SDLK_9: mSimulation.reset(loadSimulation9Parameters()); break;
                            // case SDLK_0: mSimulation.reset(loadSimulation0Parameters()); break;
                            case SDLK_l: mSimulation.selectFilter(0); break;
                            case SDLK_e: mSimulation.selectFilter(1); break;
                            case SDLK_u: mSimulation.selectFilter(2); break;
                            case SDLK_o: mSimulation.selectFilter(3); break;
                            case SDLK_c: mSimulation.toggleSensorCompass(); break;
                            case SDLK_i: mSimulation.toggleSensorIMU(); break;
                            case SDLK_g: mSimulation.toggleSensorGPS(); break;
                            case SDLK_d: mSimulation.toggleSensorLidar(); break;
                            case SDLK_w: mSimulation.toggleSensorWheelEncoder(); break;
                        }
                    }
                }
            }

            // Save Simulation Data of the current filter
            filter_simulation_data[filter_type].filter_name = mSimulation.m_selected_filter->getName();
            filter_simulation_data[filter_type].vehicle_path = mSimulation.m_vehicle_position_history;
            filter_simulation_data[filter_type].filter_path = mSimulation.m_filter_position_history;
            filter_simulation_data[filter_type].position_error = mSimulation.m_filter_error_position_history;
            filter_simulation_data[filter_type].heading_error = mSimulation.m_filter_error_heading_history;
            filter_simulation_data[filter_type].position_rmse = calculateRMSE(mSimulation.m_filter_error_position_history);
            filter_simulation_data[filter_type].heading_rmse = calculateRMSE(mSimulation.m_filter_error_heading_history);
            filter_simulation_data[filter_type].max_position_error = mSimulation.m_max_position_error;
            filter_simulation_data[filter_type].avg_cpu_time = mSimulation.m_cpu_time_avg;
        }
        // Save plots
        saveProfileSimData(filter_simulation_data, sim_param.profile_name);
        // Save metrics
        saveMetricsData(filter_simulation_data, sim_param.profile_name);
    }





    // Destroy Renderer
    mDisplay.destroyRenderer();

    // Unload SDL
    TTF_Quit();
    SDL_Quit();

    return 0;
}

SimulationParams loadSimulation1Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "1 - Ideal Conditions";
    sim_params.car_initial_velocity = 5;
    sim_params.car_initial_psi = 0;
    sim_params.end_time = 500;

    SensorsProfile sensors_config_1;
    sensors_config_1.accel_noise_std = 0.0;
    sensors_config_1.gyro_noise_std = 0.0;
    sensors_config_1.compass_noise_std = 0.0;
    sensors_config_1.gps_position_noise_std = 0.0;
    sensors_config_1.lidar_range_noise_std = 0.0;
    sensors_config_1.lidar_theta_noise_std = 0.0;
    sensors_config_1.wheelspeed_noise_std = 0.0;

    sim_params.list_sensors_profile.push(sensors_config_1);

    sim_params.car_commands.emplace_back(new MotionCommandEightShape(500, 4, 250));
    return sim_params;
}

SimulationParams loadSimulation2Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "2 - White Noise";
    sim_params.car_initial_velocity = 5;
    sim_params.car_initial_psi = 0;
    sim_params.end_time = 500;

    SensorsProfile sensors_config_1;

    sim_params.list_sensors_profile.push(sensors_config_1);

    sim_params.car_commands.emplace_back(new MotionCommandEightShape(500, 4, 250));
    return sim_params;
}
//
SimulationParams loadSimulation3Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "3 - Outliers in Measurement";
    sim_params.end_time = 500;



    SensorsProfile sensors_config_1;
    sensors_config_1.gps_error_probability = 0.1;
    sensors_config_1.wheelspeed_error_probability = 0.1;
    sensors_config_1.compass_error_probability = 0.05;
    sensors_config_1.imu_error_probability = 0.0;
    sensors_config_1.lidar_error_probability = 0.1;

    sim_params.list_sensors_profile.push(sensors_config_1);


    sim_params.car_initial_velocity = 5;
    sim_params.car_initial_psi = 0;
    sim_params.car_commands.emplace_back(new MotionCommandEightShape(500, 4, 250));
    return sim_params;
}

SimulationParams loadSimulation4Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "4 - Displacement and Drift of Data";
    sim_params.end_time = 500;

    SensorsProfile sensors_config_1;
    sensors_config_1.gyro_bias = 0.15;
    sensors_config_1.wheelspeed_scaling_factor = 1.1;
    sensors_config_1.compass_bias = 0.15;

    sim_params.list_sensors_profile.push(sensors_config_1);

    sim_params.car_initial_velocity = 5;
    sim_params.car_initial_psi = 0;
    sim_params.car_commands.emplace_back(new MotionCommandEightShape(500, 4, 250));

    return sim_params;
}

SimulationParams loadSimulation5Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "5 - 8 Shape Profile";
    sim_params.end_time = 500;

    SensorsProfile sensors_config_1;

    sim_params.list_sensors_profile.push(sensors_config_1);

    sim_params.car_commands.emplace_back(new MotionCommandEightShape(500, 4, 250));
    return sim_params;
}

SimulationParams loadSimulation6Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "6 - Complex Trajectories";
    sim_params.end_time = 200;

    SensorsProfile sensors_config_1;

    sim_params.list_sensors_profile.push(sensors_config_1);


    // sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,0,2));
    // sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,0,6));
    // sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,100,10));
    // sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,100,6));
    // sim_params.car_commands.emplace_back(new MotionCommandMoveTo(0,0,4));
    sim_params.car_commands.emplace_back(new MotionCommandEightShape(120, 10, 50));

    return sim_params;
}

SimulationParams loadSimulation7Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "7 - Dataloss";
    sim_params.end_time = 120.0;


    SensorsProfile sensors_config_1;
    sensors_config_1.duration = 5.0;
    sensors_config_1.gps_enabled = false;
    sensors_config_1.lidar_enabled = false;
    sensors_config_1.imu_enabled = false;
    // sensors_config_1.compass_enabled = false;

    SensorsProfile sensors_config_2;
    sensors_config_2.duration = 5.0;

    for (int i = 0; i < 12; ++i)
    {
        sim_params.list_sensors_profile.push(sensors_config_1);
        sim_params.list_sensors_profile.push(sensors_config_2);
    }

    sim_params.car_commands.emplace_back(new MotionCommandEightShape(120, 10, 250));

    return sim_params;
}
//
//
SimulationParams loadSimulation8Parameters()
{
    SimulationParams sim_params;
    sim_params.profile_name = "8 - Increasing Sensor Noise";
    sim_params.end_time = 300.0;


    SensorsProfile sensors_config_1;
    sensors_config_1.duration = 10.0;

    for (int i = 0; i < sim_params.end_time / sensors_config_1.duration; ++i)
    {
        SensorsProfile sensors_config_base;
        double noise_percent = 1.0 * i  + 1.0;
        sensors_config_1.gps_position_noise_std = sensors_config_base.gps_position_noise_std * noise_percent;
        sensors_config_1.lidar_range_noise_std = sensors_config_base.lidar_range_noise_std * noise_percent;
        sensors_config_1.lidar_theta_noise_std = sensors_config_base.lidar_theta_noise_std * noise_percent;
        sensors_config_1.wheelspeed_noise_std = sensors_config_base.wheelspeed_noise_std * noise_percent;
        sensors_config_1.compass_noise_std = sensors_config_base.compass_noise_std * noise_percent;
        sensors_config_1.gyro_noise_std = sensors_config_base.gyro_noise_std * noise_percent;
        sensors_config_1.accel_noise_std = sensors_config_base.accel_noise_std * noise_percent;
        sim_params.list_sensors_profile.push(sensors_config_1);
    }

    sim_params.car_commands.emplace_back(new MotionCommandEightShape(sim_params.end_time, 10, 250));

    return sim_params;
}
//
// SimulationParams loadSimulation9Parameters()
// {
//     SimulationParams sim_params;
//     sim_params.profile_name = "9 - CAPSTONE";
//     sim_params.lidar_enabled = true;
//     sim_params.end_time = 500;
//     sim_params.car_initial_x = 400;
//     sim_params.car_initial_y = -400;
//     sim_params.car_initial_velocity = 0;
//     sim_params.car_initial_psi = M_PI/180.0 * -90.0;
//     sim_params.gps_error_probability = 0.05;
//     sim_params.gps_denied_x = 250.0;
//     sim_params.gps_denied_y = -250.0;
//     sim_params.gps_denied_range = 100.0;
//     sim_params.gyro_bias = -3.1/180.0*M_PI;
//     sim_params.car_commands.emplace_back(new MotionCommandStraight(3,-2));
//     sim_params.car_commands.emplace_back(new MotionCommandTurnTo(M_PI/180.0 * 90.0,-2));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(400,-300,5));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(350,-300,2));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-250,7));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-300,5));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(250,-250,5));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(250,-300,5));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,-250,5));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,-300,5));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,-150,2));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(100,-100,-2));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(200,0,7));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-100,5));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(300,-300,7));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(400,-300,3));
//     sim_params.car_commands.emplace_back(new MotionCommandMoveTo(400,-400,1));
//     return sim_params;
// }
//
// SimulationParams loadSimulation0Parameters()
// {
//     SimulationParams sim_params = loadSimulation9Parameters();
//     sim_params.profile_name = "0 - CAPSTONE BONUS (with No Lidar Data Association)";
//     sim_params.lidar_id_enabled = false;
//     return sim_params;
// }