#include <iostream>

#include <gtsam/base/debug.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

int main()
{
    SETDEBUG("ISAM2 recalculate", true);
    SETDEBUG("IncrementalFixedLagSmoother update", true);

    gtsam::ISAM2Params isam2_params;
    isam2_params.findUnusedFactorSlots = true;
    isam2_params.enableRelinearization = true;
    isam2_params.relinearizeSkip       = 1;
    // isam2_params.optimizationParams    = gtsam::ISAM2DoglegParams();
    // isam2_params.factorization         = gtsam::ISAM2Params::Factorization::QR;
    isam2_params.print("isam2 params : ");

    gtsam::IncrementalFixedLagSmoother incremental_fixed_lag_smoother(2, isam2_params);
    gtsam::ISAM2                       isam2(isam2_params);

    gtsam::NonlinearFactorGraph nonlinear_factor_graph;
    gtsam::Values               initial_values;

    gtsam::Pose3                             measure(gtsam::Rot3::Identity(), gtsam::Point3(1, 0, 0));
    gtsam::noiseModel::Isotropic::shared_ptr noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);

    gtsam::Symbol X0('X', 0);
    gtsam::Symbol X1('X', 1);
    initial_values.insert(X0, gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0)));
    initial_values.insert(X1, gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(1, 0, 0)));

    gtsam::PriorFactor<gtsam::Pose3>   prior_factor(X0, gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, 0)),
                                                    noise_model);
    gtsam::BetweenFactor<gtsam::Pose3> between_factor_01(X0, X1, measure, noise_model);

    nonlinear_factor_graph.add(prior_factor);
    nonlinear_factor_graph.add(between_factor_01);

    std::cout << "*********************** isam2 **************************" << std::endl;
    gtsam::ISAM2Result isam2_result      = isam2.update(nonlinear_factor_graph, initial_values);
    gtsam::Values      calculated_values = isam2.calculateEstimate();
    isam2_result.print("isam2_result : \n ");
    calculated_values.print("isam2 calculated_values : ");

    std::cout << "*********************** incremental_fixed_lag_smoother **************************" << std::endl;
    incremental_fixed_lag_smoother.update(nonlinear_factor_graph, initial_values);
    calculated_values = incremental_fixed_lag_smoother.calculateEstimate();
    incremental_fixed_lag_smoother.getISAM2Result().print("incremental fixed lag smoother result : \n");
    calculated_values.print("incremental_fixed_lag_smoother calculated_values : ");

    return 0;
}
