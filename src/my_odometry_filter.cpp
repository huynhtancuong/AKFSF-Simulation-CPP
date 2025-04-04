// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "OdometryFilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double INIT_POS_STD = 1.0;
constexpr double INIT_VEL_STD = 1.0;
constexpr double INIT_THETA_STD = 1.0;
constexpr double ACCEL_STD = 0.0;
constexpr double GPS_POS_STD = 3.0;

// -------------------------------------------------- //

double delta_t = 0.0; // Time step for handling wheel speed measurements

void OdometryFilter::predictionStep(double dt)
{
    delta_t = dt;
}

void OdometryFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    if(isInitialised())
    {

    }
    else
    {
        // State : [X, Y, theta, V]
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state << meas.x, meas.y, 0, 0;

        setState(state);
        setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }        
}


void OdometryFilter::handleWheelsSpeedMeasurement(WheelsSpeedMeasurement meas) {
    if (isInitialised())
    {
        VectorXd state = getState();

        double dt = delta_t;

        double x = state[0];
        double y = state[1];
        double theta = state[2];

        double Vr = meas.right_wheel_vel;
        double Vl = meas.left_wheel_vel;
        double l = meas.base_wheel_distance;

        double V = (Vr + Vl) / 2.0;
        double omega = (Vr - Vl) / l;

        double new_x = x + V * dt * cos(theta);
        double new_y = y + V * dt * sin(theta);
        double new_theta = wrapAngle(theta + omega * dt);

        state << new_x, new_y, new_theta, V;

        setState(state);
    }
}

void OdometryFilter::handleCompassMeasurement(CompassMeasurement meas)
{

    if (isInitialised())
    {

        VectorXd state = getState();

        double theta = meas.theta;

        state(2) = wrapAngle(theta);

        setState(state);

    }
}

Matrix2d OdometryFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState OdometryFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,VX,VY]
        double psi = std::atan2(state[3],state[2]);
        double V = std::sqrt(state[2]*state[2] + state[3]*state[3]);
        return VehicleState(state[0],state[1],psi,V);
    }
    return VehicleState();
}

void OdometryFilter::predictionStep(IMUMeasurement accel, double dt){ predictionStep(dt); }
void OdometryFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map){}
void OdometryFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map){}

