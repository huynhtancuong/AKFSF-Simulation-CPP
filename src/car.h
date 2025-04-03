#ifndef INCLUDE_AKFSFSIM_CAR_H
#define INCLUDE_AKFSFSIM_CAR_H

#include <queue>
#include <cmath>
#include "display.h"
#include "utils.h"

struct VehicleState
{
    double x,y,psi,V,accel;
    double yaw_rate;
    VehicleState():x(0.0), y(0.0), psi(0.0), V(0.0), yaw_rate(0.0) {}
    VehicleState(double setX, double setY, double setPsi, double setV):
        x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(0.0) {}
    VehicleState(double setX, double setY, double setPsi, double setV, double setPsiDot):
        x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(setPsiDot) {}
    VehicleState(double set_x, double set_y, double psi, double set_v, double omega, double accel):
        x(set_x), y(set_y), psi(psi), V(set_v), yaw_rate(omega), accel(accel) {}
};

class MotionCommandBase
{
    public:
        MotionCommandBase():m_left_wheel_velocity(0.0),m_right_wheel_velocity(0.0){}
        virtual void startCommand(double time, VehicleState state){m_start_time = time;m_start_state = state;}
        virtual void endCommand(double time, double dt, VehicleState state){}
        virtual bool update(double time, double dt, VehicleState state){return false;}
        virtual double getLeftWheelVelocityCommand(){return m_left_wheel_velocity;}
        virtual double getRightWheelVelocityCommand(){return m_right_wheel_velocity;}
    protected:
        double m_left_wheel_velocity, m_right_wheel_velocity, m_start_time;
        VehicleState m_start_state;
};

class MotionCommandStraight : public MotionCommandBase
{
    public:
    MotionCommandStraight(double command_time, double command_velocity):m_command_time(command_time),m_command_velocity(command_velocity){}
    bool update(double time, double dt, VehicleState state)
    {
        m_left_wheel_velocity = m_command_velocity;
        m_right_wheel_velocity = m_command_velocity;
        return time > (m_start_time + m_command_time);
    }
    private:
        double m_command_time, m_command_velocity;
};

class MotionCommandTurnTo : public MotionCommandBase
{
    public:
    MotionCommandTurnTo(double command_heading, double command_velocity):m_command_heading(command_heading),m_command_velocity(command_velocity){}
    bool update(double time, double dt, VehicleState state)
    {
        double angle_error = wrapAngle(m_command_heading - state.psi);
        double m_steering_command = angle_error * (std::signbit(state.V) ? -1.0 : 1.0);
        m_left_wheel_velocity = m_command_velocity - (m_steering_command * state.V / 2.0);
        m_right_wheel_velocity = m_command_velocity + (m_steering_command * state.V / 2.0);
        return std::fabs(angle_error) < 0.001;
    }
    private:
        double m_command_heading, m_command_velocity;
};

class MotionCommandMoveTo : public MotionCommandBase
{
    public:
    MotionCommandMoveTo(double command_x, double command_y, double command_velocity):m_command_x(command_x),m_command_y(command_y),m_command_velocity(command_velocity){}
    bool update(double time, double dt, VehicleState state)
    {
        double delta_x = m_command_x - state.x;
        double delta_y = m_command_y - state.y;
        double range = sqrt(delta_x*delta_x + delta_y*delta_y);
        double angle_command = atan2(delta_y,delta_x);
        double psi = wrapAngle(state.psi - (std::signbit(state.V) ?M_PI:0.0));
        double angle_error = wrapAngle(angle_command - psi);
        double steering_command = angle_error * (std::signbit(state.V)?-1.0:1.0);
        m_left_wheel_velocity = m_command_velocity - (steering_command * state.V / 2.0);
        m_right_wheel_velocity = m_command_velocity + (steering_command * state.V / 2.0);
        return (range < 5.0);
    }
    private:
        double m_command_x, m_command_y, m_command_velocity;
};

class DifferentialDriveMobileRobot {
public:
    DifferentialDriveMobileRobot() : m_initial_state(VehicleState(0, 0, 0, 0)), m_wheel_base(4.0) { reset(); }
    DifferentialDriveMobileRobot(double x0, double y0, double psi0, double V0) : m_initial_state(VehicleState(x0, y0, psi0, V0)), m_wheel_base(4.0) { reset(); }

    void reset() {
        m_current_state = m_initial_state;
        m_left_wheel_velocity = 0.0;
        m_right_wheel_velocity = 0.0;
    }

    void reset(VehicleState state) {
        m_initial_state = state;
        reset();
    }

    void update(double dt) {
        double v = (m_left_wheel_velocity + m_right_wheel_velocity) / 2.0;
        double omega = (m_right_wheel_velocity - m_left_wheel_velocity) / m_wheel_base;
        double accel = (v - m_current_state.V) / dt;

        double cosPsi = cos(m_current_state.psi);
        double sinPsi = sin(m_current_state.psi);
        double x = m_current_state.x + v * cosPsi * dt;
        double y = m_current_state.y + v * sinPsi * dt;
        double psi = wrapAngle(m_current_state.psi + omega * dt);

        m_current_state = VehicleState(x, y, psi, v, omega, accel);
    }


    void setWheelVelocities(double left, double right) {
        m_left_wheel_velocity = left;
        m_right_wheel_velocity = right;
    }

    VehicleState getVehicleState() const { return m_current_state; }

private:
    VehicleState m_current_state;
    VehicleState m_initial_state;

    double m_left_wheel_velocity;
    double m_right_wheel_velocity;

    double m_wheel_base;
};

class Car
{
    public:

        Car():m_vehicle_model(),m_current_command(nullptr)
        {
            // Create Display Geometry
            m_car_lines_body = {{1,0},{0.707,0.707},{0,1},{-0.707,0.707},{-1,0},{-0.707,-0.707},{0,-1},{0.707,-0.707},{1,0}};
            m_marker_lines = {{{0.5,0.5},{-0.5,-0.5}}, {{0.5,-0.5},{-0.5,0.5}}, {{0,0},{3.5,0}}};
            m_wheel_lines = {{-0.6,0.3},{0.6, 0.3},{0.6, -0.3},{-0.6, -0.3},{-0.6, 0.3}};
            m_wheel_left_offset = Vector2(0, -1.6);
            m_wheel_right_offset = Vector2(0, 1.6);
        }

        void reset(double x0, double y0, double psi0, double V0)
        {
            m_vehicle_model.reset(VehicleState(x0,y0,psi0,V0));
            while (!m_vehicle_commands.empty()){m_vehicle_commands.pop();}
            m_current_command = nullptr;
        }

        void addVehicleCommand(MotionCommandBase* cmd)
        {
            if (cmd != nullptr){m_vehicle_commands.push(cmd);}
        }

        VehicleState getVehicleState() const {return m_vehicle_model.getVehicleState();}

        bool update(double time, double dt)
        {
            // Update Command
            if(m_current_command == nullptr && !m_vehicle_commands.empty())
            {
                m_current_command = m_vehicle_commands.front();
                m_vehicle_commands.pop();
                m_current_command->startCommand(time, m_vehicle_model.getVehicleState());
            }

            // Run Command
            if (m_current_command != nullptr)
            {
                bool cmd_complete = m_current_command->update(time, dt, m_vehicle_model.getVehicleState());
                m_vehicle_model.setWheelVelocities(m_current_command->getLeftWheelVelocityCommand(), m_current_command->getRightWheelVelocityCommand());
                if(cmd_complete){m_current_command = nullptr;}
            }
            else
            {
                m_vehicle_model.setWheelVelocities(0.0, 0.0);
            }

            // Update Vehicle
            m_vehicle_model.update(dt);

            return true;
        }

        void render(Display& disp)
        {
            double carPsiOffset = m_vehicle_model.getVehicleState().psi;
            Vector2 carPosOffset = Vector2(m_vehicle_model.getVehicleState().x, m_vehicle_model.getVehicleState().y);
            
            disp.setDrawColour(0,255,0);
            disp.drawLines(transformPoints(scalePoints(m_car_lines_body, 2.5), carPosOffset, carPsiOffset));
            disp.drawLines(transformPoints(m_marker_lines, carPosOffset, carPsiOffset));

            disp.setDrawColour(0,201,0);
            disp.drawLines(transformPoints(offsetPoints(m_wheel_lines, m_wheel_left_offset), carPosOffset, carPsiOffset));
            disp.drawLines(transformPoints(offsetPoints(m_wheel_lines, m_wheel_right_offset), carPosOffset, carPsiOffset));
        }

    private:

        DifferentialDriveMobileRobot m_vehicle_model;
        MotionCommandBase* m_current_command;
        std::queue<MotionCommandBase*> m_vehicle_commands;

        std::vector<Vector2> m_car_lines_body;
        std::vector<Vector2> m_wheel_lines;
        std::vector<std::vector<Vector2>> m_marker_lines;
        Vector2 m_wheel_fl_offset, m_wheel_fr_offset, m_wheel_left_offset, m_wheel_right_offset;
};


#endif  // INCLUDE_AKFSFSIM_CAR_H