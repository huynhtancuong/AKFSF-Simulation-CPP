// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Unscented Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 0.5;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 2;
constexpr double INIT_PSI_STD = 5.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

// ----------------------------------------------------------------------- //
// USEFUL HELPER FUNCTIONS
VectorXd normaliseState(VectorXd state)
{
    state(2) = wrapAngle(state(2));
    return state;
}
VectorXd normaliseLidarMeasurement(VectorXd meas)
{
    meas(1) = wrapAngle(meas(1));
    return meas;
}
std::vector<VectorXd> generateSigmaPoints(VectorXd state, MatrixXd cov)
{
    std::vector<VectorXd> sigmaPoints;

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE
    const unsigned int n = state.size();
    const int kappa = 3 - n;

    sigmaPoints.reserve(2*n+1);

    sigmaPoints.push_back(state);

    MatrixXd L = ((n+kappa) * cov).llt().matrixL();
    for (Eigen::Index i = 0; i<n; i++) {
        VectorXd dState = L.col(i);

        sigmaPoints.emplace_back(state + dState);
        sigmaPoints.emplace_back(state - dState);
    }

    // ----------------------------------------------------------------------- //

    return sigmaPoints;
}

std::vector<double> generateSigmaWeights(unsigned int numStates)
{
    std::vector<double> weights;

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE
    weights.reserve(2*numStates + 1);

    const double kappa = 3.0 - numStates;
    const double w0 = kappa / (numStates + kappa);
    const double wi = 0.5 / (numStates + kappa);
    weights.push_back(w0);
    for (size_t i = 0; i< 2*numStates; i++) {
        weights.push_back(wi);
    }

    // ----------------------------------------------------------------------- //

    return weights;
}

VectorXd lidarMeasurementModel(VectorXd aug_state, double beaconX, double beaconY)
{
    VectorXd z_hat = VectorXd::Zero(2);

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE
    double px = aug_state(0);
    double py = aug_state(1);
    double psi = aug_state(2);
    double range_noise = aug_state(4);
    double theta_noise = aug_state(5);
    double Lx = beaconX;
    double Ly = beaconY;
    double delta_x = Lx - px;
    double delta_y = Ly - py;

    double r_hat = sqrt( pow(delta_x, 2) + pow(delta_y, 2) ) + range_noise;
    double theta_hat = atan2(delta_y, delta_x) - psi + theta_noise;

    z_hat << r_hat, theta_hat;


    // ----------------------------------------------------------------------- //

    return z_hat;
}

VectorXd vehicleProcessModel(VectorXd aug_state, double psi_dot, double dt)
{
    VectorXd new_state = VectorXd::Zero(4);

    // ----------------------------------------------------------------------- //
    // ENTER YOUR CODE HERE
    new_state(0) = aug_state(0) + dt * aug_state(3) * cos(aug_state(2));
    new_state(1) = aug_state(1) + dt * aug_state(3) * sin(aug_state(2));
    new_state(2) = aug_state(2) + dt * (psi_dot + aug_state(4));
    new_state(3) = aug_state(3) + dt * aug_state(5);

    // ----------------------------------------------------------------------- //

    return new_state;
}
// ----------------------------------------------------------------------- //

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the 
        // section below.
        // HINT: Use the normaliseState() and normaliseLidarMeasurement() functions
        // to always keep angle values within correct range.
        // HINT: Do not normalise during sigma point calculation!
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1) // Check that we have a valid beacon match
        {

            const unsigned int n_v = 2; // Size of measurement noise
            const unsigned int n_x = state.size(); // Size of state
            const unsigned int n_z = 2; // Size of measurement
            const unsigned int n_aug = n_v + n_x;

            // Create measurement vector and measurement noise cov
            VectorXd z = VectorXd::Zero(n_z);
            z << meas.range, meas.theta;

            Matrix2d R;
            R <<    LIDAR_RANGE_STD * LIDAR_RANGE_STD, 0,
                    0, LIDAR_THETA_STD*LIDAR_THETA_STD;

            // Augment state vector and cov matrix with measurement noise
            VectorXd x_aug = VectorXd::Zero(n_aug);
            x_aug.head(n_x) = state;

            MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);
            P_aug.topLeftCorner(n_x, n_x) = cov;
            P_aug.bottomRightCorner(n_v, n_v) = R;

            // Create sigma points and weights
            const auto aug_state_sigma_points = generateSigmaPoints(x_aug, P_aug);
            const auto sigma_weights = generateSigmaWeights(n_aug);

            // Transform the sigma points with measurement model to get the measurement predictions
            std::vector<VectorXd> z_predictions;
            for (const auto& aug_state : aug_state_sigma_points) {
                auto z_prediction = lidarMeasurementModel(aug_state, map_beacon.x, map_beacon.y);
                z_predictions.push_back(z_prediction);
            }

            // Calculate Measurement Mean
            VectorXd z_mean = VectorXd::Zero(n_z);
            for (size_t i = 0; i< z_predictions.size(); i++) {
                z_mean += sigma_weights.at(i) * z_predictions.at(i);
            }

            // Calculate Innovation
            VectorXd y = normaliseLidarMeasurement(z - z_mean);

            // Calculate cov matrix of Innovation
            MatrixXd S = MatrixXd::Zero(n_z, n_z);
            for (size_t i = 0; i < z_predictions.size(); i++) {
                VectorXd diff = z_predictions.at(i) - z_mean;
                diff = normaliseLidarMeasurement(diff);
                S += sigma_weights.at(i) * diff * diff.transpose();
            }

            // Calculate Cross Covariance
            MatrixXd Pxz = MatrixXd::Zero(n_x, n_z);
            for (size_t i = 0; i < aug_state_sigma_points.size(); i++) {
                VectorXd state_sigma_point = aug_state_sigma_points.at(i).head(n_x);

                VectorXd x_diff = normaliseState(state_sigma_point - state);
                VectorXd z_diff = normaliseLidarMeasurement( z_predictions.at(i) - z_mean );
                Pxz += sigma_weights.at(i) * x_diff * z_diff.transpose();
            }
            // std::cout << "Pxz:\n" << Pxz << std::endl;

            // Update state and cov
            MatrixXd K = Pxz * S.inverse();
            // std::cout << "K:\n" << K << std::endl;
            state = state + K * y;
            cov = cov - K * S * K.transpose();

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
        // HINT: Use the normaliseState() function to always keep angle values within correct range.
        // HINT: Do NOT normalise during sigma point calculation!
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        Matrix2d Q;
        Q <<    GYRO_STD * GYRO_STD, 0,
                0, ACCEL_STD * ACCEL_STD;

        const unsigned int n_x = state.size();
        const unsigned int n_w = 2;
        const unsigned int n_aug = n_x + n_w;

        VectorXd x_aug = VectorXd::Zero(n_aug);
        x_aug.head(n_x) = state;

        MatrixXd P_aug = MatrixXd::Zero(n_aug, n_aug);
        P_aug.topLeftCorner(n_x,n_x) = cov;
        P_aug.bottomRightCorner(n_w, n_w) = Q;

        // Generate sigma points and their weights
        auto sigmaPoints = generateSigmaPoints(x_aug, P_aug);
        auto weights = generateSigmaWeights(n_aug);

        // Transform sigma points with vehicle process model
        std::vector<VectorXd> sigma_points_predict;
        for (const auto& sigmaPoint: sigmaPoints) {
            auto new_state = vehicleProcessModel(sigmaPoint, gyro.psi_dot, dt);
            sigma_points_predict.push_back(new_state);
        }

        // Calculate the mean
        state.setZero();
        for (size_t i = 0; i < sigmaPoints.size(); i++) {
            state += weights.at(i) * sigma_points_predict.at(i);
        }
        state = normaliseState(std::move(state));

        // Calculate the covariance matrix
        cov.setZero();
        for (size_t i = 0; i < sigmaPoints.size(); i++) {
            VectorXd diff = normaliseState(sigma_points_predict.at(i) - state);
            cov += weights.at(i) * diff * diff.transpose();
        }

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    } 
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the UKF update state would just produce the same result.
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2,4);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,meas.y;
        H << 1,0,0,0,0,1,0,0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        state = state + K*y;
        cov = (MatrixXd::Identity(4,4) - K*H) * cov;

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // You may modify this initialisation routine if you can think of a more
        // robust and accuracy way of initialising the filter.
        // ----------------------------------------------------------------------- //
        // YOU ARE FREE TO MODIFY THE FOLLOWING CODE HERE

        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state(0) = meas.x;
        state(1) = meas.y;
        cov(0,0) = GPS_POS_STD*GPS_POS_STD;
        cov(1,1) = GPS_POS_STD*GPS_POS_STD;
        cov(2,2) = INIT_PSI_STD*INIT_PSI_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

        setState(state);
        setCovariance(cov);

        // ----------------------------------------------------------------------- //
    }             
}

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
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
