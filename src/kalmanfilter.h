#ifndef INCLUDE_AKFSFSIM_KALMANFILTER_H
#define INCLUDE_AKFSFSIM_KALMANFILTER_H

#include <vector>
#include <Eigen/Dense>

#include "car.h"
#include "sensors.h"
#include "beacons.h"

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;

class KalmanFilterBase
{
public:
    KalmanFilterBase(std::string set_name):m_initialised(false), m_name(set_name){}
    virtual ~KalmanFilterBase(){}
    void reset(){m_initialised = false;}
    bool isInitialised() const {return m_initialised;}

    virtual VehicleState getVehicleState() = 0;
    virtual Matrix2d getVehicleStatePositionCovariance() = 0;

    virtual void predictionStep(double dt) = 0;

    virtual void predictionStep(IMUMeasurement imu_meas, double dt) = 0;

    virtual void handleWheelsSpeedMeasurement(WheelsSpeedMeasurement meas) = 0;
    virtual void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) = 0;
    virtual void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) = 0;
    virtual void handleGPSMeasurement(GPSMeasurement meas) = 0;
    virtual void handleCompassMeasurement(CompassMeasurement meas) = 0;
    virtual std::string getName() const {return m_name;}


protected:
    VectorXd getState() const {return m_state;}
    MatrixXd getCovariance()const {return m_covariance;}
    void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
    void setCovariance(const MatrixXd& cov ){m_covariance = cov;}

private:
    bool m_initialised;
    VectorXd m_state;
    MatrixXd m_covariance;
    std::string m_name;
};

class KalmanFilterLKF : public KalmanFilterBase
{
public:
    KalmanFilterLKF(std::string name): KalmanFilterBase(name){}
    VehicleState getVehicleState() override;
    Matrix2d getVehicleStatePositionCovariance() override;

    void predictionStep(double dt) override;
    void predictionStep(IMUMeasurement imu_meas, double dt) override;

    void handleWheelsSpeedMeasurement(WheelsSpeedMeasurement meas) override;
    void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) override;
    void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) override;
    void handleGPSMeasurement(GPSMeasurement meas) override;
    void handleCompassMeasurement(CompassMeasurement meas) override;

};

class KalmanFilterEKF : public KalmanFilterBase
{
public:
    KalmanFilterEKF(std::string name): KalmanFilterBase(name){}
    VehicleState getVehicleState() override;
    Matrix2d getVehicleStatePositionCovariance() override;


    void predictionStep(double dt) override;
    void predictionStep(IMUMeasurement imu_meas, double dt) override;

    void handleWheelsSpeedMeasurement(WheelsSpeedMeasurement meas) override;
    void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) override;
    void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) override;
    void handleGPSMeasurement(GPSMeasurement meas) override;
    void handleCompassMeasurement(CompassMeasurement meas) override;

};

class KalmanFilterUKF : public KalmanFilterBase
{
public:
    KalmanFilterUKF(std::string name): KalmanFilterBase(name){}

    VehicleState getVehicleState() override;
    Matrix2d getVehicleStatePositionCovariance() override;

    void predictionStep(double dt) override;
    void predictionStep(IMUMeasurement imu_meas, double dt) override;

    void handleWheelsSpeedMeasurement(WheelsSpeedMeasurement meas) override;
    void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) override;
    void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) override;
    void handleGPSMeasurement(GPSMeasurement meas) override;
    void handleCompassMeasurement(CompassMeasurement meas) override;

};

class OdometryFilter : public KalmanFilterBase
{
public:
    OdometryFilter(std::string name): KalmanFilterBase(name){}

    VehicleState getVehicleState() override;
    Matrix2d getVehicleStatePositionCovariance() override;

    void predictionStep(double dt) override;
    void predictionStep(IMUMeasurement imu_meas, double dt) override;

    void handleWheelsSpeedMeasurement(WheelsSpeedMeasurement meas) override;
    void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map) override;
    void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map) override;
    void handleGPSMeasurement(GPSMeasurement meas) override;
    void handleCompassMeasurement(CompassMeasurement meas) override;

};

#endif  // INCLUDE_AKFSFSIM_KALMANFILTER_H