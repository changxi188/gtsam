/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testImuPreintegration.cpp
 * @brief  Unit tests for IMU Preintegration
 * @author Russell Buchanan
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <tests/ImuMeasurement.h>

#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
/**
 * \brief Uses the GTSAM library to perform IMU preintegration on an
 * acceleration input.
 */
TEST(TestImuPreintegration, LoadedSimulationData) {
  Eigen::Vector3d finalPos;

  vector<ImuMeasurement> imuMeasurements;

  double accNoiseSigma = 0.001249;
  double accBiasRwSigma = 0.000106;
  double gyrNoiseSigma = 0.000208;
  double gyrBiasRwSigma = 0.000004;
  double integrationCovariance = 1e-8;
  double biasAccOmegaInt = 1e-5;

  double gravity = 9.81;
  double rate = 400.0;  // Hz

  string inFileString = findExampleDataFile("quadraped_imu_data.csv");
  ofstream outputFile;
  outputFile.open("imu_preint_output.csv", ios::out);
  ifstream inputFile(inFileString);
  string line;
  while (getline(inputFile, line)) {
    stringstream ss(line);
    string str;
    vector<double> results;
    while (getline(ss, str, ',')) {
      results.push_back(atof(str.c_str()));
    }
    ImuMeasurement measurement;
    measurement.dt = static_cast<size_t>(1e9 * (1 / rate));
    measurement.time = results[2];
    measurement.I_a_WI = {results[29], results[30], results[31]};
    measurement.I_w_WI = {results[17], results[18], results[19]};
    imuMeasurements.push_back(measurement);

    // cout << "IMU measurement " << measurement << endl;
  }

  // Assume a Z-up navigation (assuming we are performing optimization in the
  // IMU frame).
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params>
      imuPreintegratedParams =
          PreintegratedCombinedMeasurements::Params::MakeSharedU(
              gravity);
  imuPreintegratedParams->accelerometerCovariance =
      I_3x3 * pow(accNoiseSigma, 2);
  imuPreintegratedParams->biasAccCovariance = I_3x3 * pow(accBiasRwSigma, 2);
  imuPreintegratedParams->gyroscopeCovariance = I_3x3 * pow(gyrNoiseSigma, 2);
  imuPreintegratedParams->biasOmegaCovariance = I_3x3 * pow(gyrBiasRwSigma, 2);
  imuPreintegratedParams->integrationCovariance = I_3x3 * integrationCovariance;
  imuPreintegratedParams->biasAccOmegaInt = I_6x6 * biasAccOmegaInt;

  // Initial state
  Pose3 priorPose;
  Vector3 priorVelocity;
  imuBias::ConstantBias priorImuBias;
  PreintegratedCombinedMeasurements imuPreintegrated;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  NavState propState;

  NavState initialNavState(priorPose, priorVelocity);

  // Bias estimated by my Algorithm
  priorImuBias = imuBias::ConstantBias(
      Eigen::Vector3d(-0.0314648, 0.0219921, 6.95945e-05),
      Eigen::Vector3d(4.88581e-08, -1.04971e-09, -0.000122868));

  // zero bias
  // priorImuBias = imuBias::ConstantBias(Eigen::Vector3d(0,0,0),
  //                                             Eigen::Vector3d(0,0,0));

  imuPreintegrated = PreintegratedCombinedMeasurements(
      imuPreintegratedParams, priorImuBias);

  // Put header row in output csv
  outputFile << "X Position,"
             << "Y Position,"
             << "Z Position,"
             << "X Velocity,"
             << "Y Velocity,"
             << "Z Velocity,"
             << "\n";

  // start at 1 to skip header
  for (size_t n = 1; n < imuMeasurements.size(); n++) {
    // integrate
    imuPreintegrated.integrateMeasurement(imuMeasurements[n].I_a_WI,
                                          imuMeasurements[n].I_w_WI, 1 / rate);
    // predict
    propState = imuPreintegrated.predict(initialNavState, priorImuBias);
    position = propState.pose().translation();
    velocity = propState.velocity();
    // cout << "IMU Position " << position.transpose() << endl;
    // cout << "IMU Velocity " << velocity.transpose() << endl;

    // Write to csv
    outputFile << to_string(position.x()) << "," << to_string(position.y())
               << "," << to_string(position.z()) << ","
               << to_string(velocity.x()) << "," << to_string(velocity.y())
               << "," << to_string(velocity.z()) << ","
               << "\n";
  }

  outputFile.close();

  Vector3 rotation = propState.pose().rotation().rpy();

  // Dont have ground truth for x and y position yet
  // DOUBLES_EQUAL(0.1, position[0], 1e-2);
  // DOUBLES_EQUAL(0.1, position[1], 1e-2);
  DOUBLES_EQUAL(0.0, position[2], 1e-2);

  DOUBLES_EQUAL(0.0, rotation[0], 1e-2);
  DOUBLES_EQUAL(0.0, rotation[1], 1e-2);
  DOUBLES_EQUAL(0.0, rotation[2], 1e-2);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
