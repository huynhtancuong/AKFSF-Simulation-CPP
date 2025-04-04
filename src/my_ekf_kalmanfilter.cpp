// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "KalmanFilterEKF.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 0.01;
constexpr double GYRO_STD = 0.01;
constexpr double WHEEL_SPEED_STD = 1.0;
constexpr double COMPASS_STD = 0.1;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;

constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_THETA_STD = 90.0/180.0 * M_PI;
constexpr double INIT_OMEGA_STD = 1.0/180.0 *M_PI;
constexpr double INIT_POS_STD = 10.0;

constexpr double PROCESS_NOISE_POS_STD = 0.01;
constexpr double PROCESS_NOISE_VEL_STD = 0.01;
constexpr double PROCESS_NOISE_THETA_STD = 0.01;
constexpr double PROCESS_NOISE_OMEGA_STD = 0.01;


constexpr bool INIT_ON_FIRST_PREDICTION = false;
// -------------------------------------------------- //

MatrixXd getProcessStateJacobianMatrix(const VectorXd& state, const double dt)
{
    size_t n = state.size();
    double theta = state[2];
    double V = state[3];
    MatrixXd F = MatrixXd::Zero(n, n);

    F <<    1, 0, -dt*V*sin(theta), dt*cos(theta), 0,
            0, 1, dt*V*cos(theta), dt*sin(theta), 0,
            0, 0, 1, 0, dt,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

    return F;
}

MatrixXd getProcessModelNoiseCovarianceMatrix()
{
    MatrixXd Q = MatrixXd::Zero(5, 5);
    Q <<    PROCESS_NOISE_POS_STD * PROCESS_NOISE_POS_STD, 0, 0, 0, 0,
            0, PROCESS_NOISE_POS_STD * PROCESS_NOISE_POS_STD, 0, 0, 0,
            0, 0, PROCESS_NOISE_THETA_STD * PROCESS_NOISE_THETA_STD, 0, 0,
            0, 0, 0, PROCESS_NOISE_VEL_STD * PROCESS_NOISE_VEL_STD, 0,
            0, 0, 0, 0, PROCESS_NOISE_OMEGA_STD * PROCESS_NOISE_OMEGA_STD;
    return Q;
}

void KalmanFilterEKF::predictionStep(double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        size_t n = state.size();
        double px = state[0];
        double py = state[1];
        double theta = state[2];
        double V = state[3];
        double omega = state[4];

        px = px + V * dt * cos(theta);
        py = py + V * dt * sin(theta);
        theta = wrapAngle(theta + omega * dt);

        MatrixXd F = getProcessStateJacobianMatrix(state, dt);
        MatrixXd Q = getProcessModelNoiseCovarianceMatrix();

        state << px, py, theta, V, omega;
        cov = F * cov * F.transpose() + Q;

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilterEKF::predictionStep(IMUMeasurement accel, double dt)
{

    if (!isInitialised() && INIT_ON_FIRST_PREDICTION)
    {

        VectorXd state = VectorXd::Zero(5);
        MatrixXd cov = MatrixXd::Zero(5, 5);

        // Assume the initial position is (X,Y) = (0,0) m
        // Assume the initial velocity is 5 m/s at 45 degrees (VX,VY) = (5*cos(45deg),5*sin(45deg)) m/s
        state << 0, 0, 45*M_PI/180.0, 5, 0;

        cov(0,0) = GPS_POS_STD*GPS_POS_STD;
        cov(1,1) = GPS_POS_STD*GPS_POS_STD;
        cov(2,2) = INIT_VEL_STD*INIT_VEL_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

        setState(state);
        setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }

    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        size_t n = state.size();
        double px = state[0];
        double py = state[1];
        double theta = state[2];
        double V = state[3];
        double omega = state[4];

        px = px + V * dt * cos(theta);
        py = py + V * dt * sin(theta);
        theta = wrapAngle(theta + omega * dt);
        V = V + accel.accel * dt;
        omega = accel.yaw_rate;

        // std::cout << accel.accel << std::endl;


        MatrixXd F = getProcessStateJacobianMatrix(state, dt);
        MatrixXd N = MatrixXd::Zero(n,n); // Process input covariance matrix
        MatrixXd Q = getProcessModelNoiseCovarianceMatrix(); // Process noise covariance matrix
        N(3,3) = ACCEL_STD * ACCEL_STD * dt * dt;
        N(4,4) = GYRO_STD * GYRO_STD * dt * dt;


        state << px, py, theta, V, omega;
        cov = F * cov * F.transpose() + Q + N;

        setState(state);
        setCovariance(cov);

    }
}

void KalmanFilterEKF::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
}

void KalmanFilterEKF::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the
        // section below.
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE
        const auto px = state[0];
        const auto py = state[1];
        const auto theta = state[2];

        Vector2d z(meas.range, meas.psi);

        const BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1)
        {
            // The map matched beacon positions can be accessed using: map_beacon.x AND map_beacon.y
            const double Lx = map_beacon.x;
            const double Ly = map_beacon.y;
            auto getRange = [] (double Lx, double Ly, double px, double py) {
                return sqrt(pow(Lx - px, 2) + pow(Ly - py, 2));
            };
            auto getBearing = [] (double Lx, double Ly, double px, double py, double theta) {
                double bearing = atan2((Ly - py) , (Lx - px)) - theta;
                bearing = wrapAngle(bearing);
                return bearing;
            };
            double predicted_range = getRange(Lx, Ly, px, py);
            double predicted_bearing = getBearing(Lx, Ly, px, py, theta);
            Vector2d z_hat(predicted_range, predicted_bearing);

            // Calculate Measurement Innovation
            VectorXd y = z - z_hat;
            y(1) = wrapAngle(y(1)); // Wrap the Heading Innovation

            // Calculate Measurement Jacobian Matrix
            MatrixXd H = MatrixXd::Zero(2, 5);
            double dx = px - Lx;
            double dy = py - Ly;
            double d = sqrt(dx*dx + dy*dy);
            H <<    dx/d, dy/d, 0, 0, 0,
                    -dy/(d*d), dx/(d*d), -1, 0, 0;

            // Measurement Noise Covariance Matrix
            Matrix2d R = Matrix2d::Zero();
            R(0, 0) = LIDAR_RANGE_STD * LIDAR_RANGE_STD;
            R(1, 1) = LIDAR_THETA_STD * LIDAR_THETA_STD;

            // Calculate Measurement Innovation Covariance
            MatrixXd S = H*cov*H.transpose() + R;

            // Calculate Kalman Gain Matrix
            MatrixXd K = cov * H.transpose() * S.inverse();

            // Update the state
            state = state + K * y;
            state(2) = wrapAngle(state(2)); // Wrap the angle to be within -pi to pi

            // Update Covariance Matrix
            MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
            cov = (I - K * H) * cov;
        }

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilterEKF::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the EKF update state would just produce the same result.
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd::Zero(2, 5);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,meas.y;
        H(0,0) = 1;
        H(1,1) = 1;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        state = state + K*y;
        MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
        cov = (I - K*H) * cov * (I - K*H).transpose() + K*R*K.transpose();

        // Check for NaN values in the updated state
        if (state.hasNaN() || cov.hasNaN()) {
            std::cerr << "Error: NaN value detected in state or covariance after GPS update step" << std::endl;
            std::cerr << "State: " << state.transpose() << std::endl;
            std::cerr << "Covariance: " << cov << std::endl;
            return;
        }

        setState(state);
        setCovariance(cov);
    }
    else
    {
        VectorXd state = VectorXd::Zero(5);
        MatrixXd cov = MatrixXd::Zero(5,5);

        state(0) = meas.x;
        state(1) = meas.y;
        state(2) = 45*M_PI/180.0;
        state(3) = 5;
        state(4) = 0;
        cov(0,0) = GPS_POS_STD*GPS_POS_STD;
        cov(1,1) = GPS_POS_STD*GPS_POS_STD;
        cov(2,2) = INIT_THETA_STD*INIT_THETA_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;
        cov(4,4) = INIT_OMEGA_STD*INIT_OMEGA_STD;

        setState(state);
        setCovariance(cov);
    } 
             
}

void KalmanFilterEKF::handleWheelsSpeedMeasurement(WheelsSpeedMeasurement meas)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        size_t n = state.size();
        MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
        double l = meas.base_wheel_distance;

        VectorXd z = VectorXd::Zero(2);
        z << meas.right_wheel_vel, meas.left_wheel_vel;

        // Measurement Jacobian Matrix
        MatrixXd H = MatrixXd::Zero(2, n);
        H <<    0, 0, 0, 1, l/2.0,
                0, 0, 0, 1, -l/2.0;

        // Measurement Noise Covariance Matrix
        MatrixXd R = MatrixXd::Zero(2, 2);
        R(0, 0) = WHEEL_SPEED_STD * WHEEL_SPEED_STD;
        R(1, 1) = WHEEL_SPEED_STD * WHEEL_SPEED_STD;

        // Calculate the Kalman Gain
        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov * H.transpose() * S.inverse();

        // Update the state and covariance
        state = state + K * y;
        cov = (I - K * H) * cov;

        state(2) = wrapAngle(state(2)); // Wrap the angle to be within -pi to pi

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilterEKF::handleCompassMeasurement(CompassMeasurement meas)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        size_t n = state.size();
        MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());


        VectorXd z = VectorXd::Zero(1);
        z << wrapAngle(meas.theta);

        // std::cout << "Heading: " << z << std::endl;

        // Measurement Jacobian Matrix
        MatrixXd H = MatrixXd::Zero(1, n);
        H <<    0, 0, 1, 0, 0;

        // Measurement Noise Covariance Matrix
        MatrixXd R = MatrixXd::Zero(1, 1);
        R(0, 0) = COMPASS_STD * COMPASS_STD;

        // Calculate the Kalman Gain
        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        y(0) = wrapAngle(y(0)); // Wrap the angle to be within -pi to pi
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov * H.transpose() * S.inverse();

        // Update the state and covariance
        state = state + K * y;
        cov = (I - K * H) * cov * (I - K * H).transpose() + K * R * K.transpose();

        // Wrap the angle to be within -pi to pi
        state(2) = wrapAngle(state(2));

        setState(state);
        setCovariance(cov);
    }
}

Matrix2d KalmanFilterEKF::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilterEKF::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,THETA,V,...]
        return VehicleState(state[0],state[1],state[2],state[3], state[4]);
    }
    return VehicleState();
}