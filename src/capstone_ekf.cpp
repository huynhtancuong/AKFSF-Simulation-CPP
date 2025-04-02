// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_GYRO_BIAS = 0.0;
constexpr double GYRO_BIAS_STD = 5.0/180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {
        handleLidarMeasurement(meas, map);
    }
}

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised()) {
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
        const auto psi = state[2];
        Vector2d z(meas.range, meas.theta);

        auto getRange = [] (double Lx, double Ly, double px, double py) {
            return sqrt(pow(Lx - px, 2) + pow(Ly - py, 2));
        };
        auto getBearing = [] (double Lx, double Ly, double px, double py, double psi) {
            double bearing = atan2((Ly - py) , (Lx - px)) - psi;
            bearing = wrapAngle(bearing);
            return bearing;
        };
        auto getPredictedMeasurement = [getBearing, getRange] (double Lx, double Ly, double px, double py, double psi) -> VectorXd {
            double r_hat = getRange(Lx, Ly, px, py);
            double theta_hat = getBearing(Lx, Ly, px, py, psi);
            Vector2d z_hat(r_hat, theta_hat);
            return z_hat;
        };

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id == -1 || map_beacon.id == -1)
        {
            /* ID Matching */
            const auto beacons = map.getBeaconsWithinRange(px, py, meas.range + 3*LIDAR_RANGE_STD);
            if (beacons.empty()) return;

            double min_innovation = std::numeric_limits<double>::max();
            for (const auto& beacon : beacons) {
                Vector2d z_hat = getPredictedMeasurement(beacon.x, beacon.y, px, py, psi);
                Vector2d y = z - z_hat;
                y(1) = wrapAngle(y(1));
                // double innovation = y.transpose() * y; // Calculate squared innovation
                double innovation = y.norm();

                if (innovation < min_innovation) {
                    min_innovation = innovation;
                    map_beacon = beacon;
                }
            }


        }

        // The map matched beacon positions can be accessed using: map_beacon.x AND map_beacon.y
        const double Lx = map_beacon.x;
        const double Ly = map_beacon.y;

        // Calculate predicted measurement
        Vector2d z_hat = getPredictedMeasurement(Lx, Ly, px, py, psi);

        // Calculate Measurement Innovation
        Vector2d y = z - z_hat;
        y(1) = wrapAngle(y(1));

        // Calculate Measurement Jacobian Matrix
        MatrixXd H(2,5);
        double d = sqrt( pow(Lx-px, 2) + pow(Ly-py,2) );

        H <<    (px - Lx)/d, (py - Ly)/d, 0, 0, 0,
                -(py - Ly)/(d*d), (px - Lx)/(d*d), -1, 0, 0;

        // Measurement Noise Covariance Matrix
        Matrix2d R = Matrix2d::Zero();
        R(0, 0) = LIDAR_RANGE_STD * LIDAR_RANGE_STD;
        R(1, 1) = LIDAR_THETA_STD * LIDAR_THETA_STD;

        // Calculate Measurement Innovation Covariance
        MatrixXd S = H*cov*H.transpose() + R;

        /* Faulty measurement test */
        bool isFaulty = false;
        // Calculate Normalised Innovation Squared
        double NIS = y.transpose()*S.inverse()*y;
        if (NIS >= 20.0) {
            isFaulty = true;
            std::cout << "LIDAR NIS Failed: " << NIS << std::endl;
        }


        if (!isFaulty) { // Update with measurement if it is not faulty
            // Calculate Kalman Gain Matrix
            MatrixXd K = cov * H.transpose() * S.inverse();

            // Update the state
            state = state + K * y;

            // Update Covariance Matrix
            MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
            cov = (I - K * H) * cov;
        }


        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the  
        // section below.
        // HINT: Assume the state vector has the form [PX, PY, PSI, V].
        // HINT: Use the Gyroscope measurement as an input into the prediction step.
        // HINT: You can use the constants: ACCEL_STD, GYRO_STD
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        // State Prediction
        VectorXd input_vec = VectorXd::Zero(5);
        double psi = state[2];
        double V = state[3];
        double wb = state[4];
        input_vec <<    V * cos(psi),
                        V * sin(psi),
                        gyro.psi_dot - wb,
                        0,
                        0;

        state = state + dt * input_vec; // Update state

        state[2] = wrapAngle(state[2]); // Normalize psi after prediction

        // Covariance Prediction
        MatrixXd F(5,5), Q(4,4), fw(5,4);
        F <<    1, 0, -dt*V*sin(psi), dt*cos(psi), 0,
                0, 1, dt*V*cos(psi), dt*sin(psi), 0,
                0, 0, 1, 0, -dt,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

        fw <<   1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, dt, 0,
                0, 0, 0, dt,
                0, 0, 0, 0;

        Q <<    0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, GYRO_STD*GYRO_STD, 0,
                0, 0, 0, ACCEL_STD*ACCEL_STD;

        cov = F * cov * F.transpose() + fw * Q * fw.transpose();

        // std::cout << "gyro_bias: " << state(4)*180/M_PI << std::endl;

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    } 
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the EKF update state would just produce the same result.
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2,5);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,meas.y;
        H <<    1,0,0,0,0,
                0,1,0,0,0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        /* GPS Faulty Test */
        // Calculate Normalised Innovation Squared
        double NIS = y.transpose()*S.inverse()*y;
        if (NIS < 10.0 ) {
            // Passed the test
            state = state + K*y;
            MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
            cov = (I - K*H) * cov;
        }
        else {
            std::cout << "GPS NIS Failed: " << NIS << std::endl;
        }



        setState(state);
        setCovariance(cov);
    }
    else
    {
        // State = [px, py, psi, V, wb], where wb is the gyro bias
        VectorXd state = VectorXd::Zero(5);
        MatrixXd cov = MatrixXd::Zero(5,5);

        state(0) = meas.x;
        state(1) = meas.y;
        state(4) = INIT_GYRO_BIAS;
        cov(0,0) = GPS_POS_STD*GPS_POS_STD;
        cov(1,1) = GPS_POS_STD*GPS_POS_STD;
        cov(2,2) = INIT_PSI_STD*INIT_PSI_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;
        cov(4,4) = GYRO_BIAS_STD*GYRO_BIAS_STD;

        setState(state);
        setCovariance(cov);
    }
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0],state[1],state[2],state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}
