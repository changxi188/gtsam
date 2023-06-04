#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2-impl.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

int main()
{
    gtsam::Key   X1 = gtsam::symbol('X', 1);
    gtsam::Key   X2 = gtsam::symbol('X', 2);
    gtsam::Pose3 p1(gtsam::Rot3::Identity(), gtsam::Point3::Identity());
    gtsam::Pose3 p2(gtsam::Rot3::AxisAngle(gtsam::Unit3(0, 0, 1), M_PI_4), gtsam::Point3(1, 0, 0));
    gtsam::Pose3 measure(gtsam::Rot3::AxisAngle(gtsam::Unit3(0, 0, 1), M_PI_4), gtsam::Point3(1, 0, 0));
    gtsam::noiseModel::Isotropic::shared_ptr noise_mode = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);

    gtsam::PriorFactor<gtsam::Pose3>   prior_factor(X1, p1, noise_mode);
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(X1, X2, measure, noise_mode);

    gtsam::Values values;
    values.insert(X1, p1);
    values.insert(X2, p2);

    gtsam::NonlinearFactorGraph nonlinear_factor_graph;
    // nonlinear_factor_graph.add(prior_factor);
    nonlinear_factor_graph.add(between_factor);

    boost::shared_ptr<gtsam::GaussianFactorGraph> gaussian_factor_graph = nonlinear_factor_graph.linearize(values);
    gtsam::VariableIndex                          affectedFactorsVarIndex(*gaussian_factor_graph);
    gtsam::FastMap<gtsam::Key, int>               constraintGroups;
    const gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrained(affectedFactorsVarIndex, constraintGroups);
    gtsam::GaussianEliminationTree etree(*gaussian_factor_graph, affectedFactorsVarIndex, ordering);
    gtsam::ISAM2Params             isam2_params;
    isam2_params.factorization = gtsam::ISAM2Params::QR;
    const auto eliminate_func  = isam2_params.getEliminationFunction();

    gtsam::ISAM2JunctionTree                       junction_tree(etree);
    const boost::shared_ptr<gtsam::ISAM2BayesTree> isam2_bayes_tree = junction_tree.eliminate(eliminate_func).first;
    return 0;
}
