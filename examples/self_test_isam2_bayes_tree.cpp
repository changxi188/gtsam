#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2-impl.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

int main()
{
    gtsam::Key                               X1 = gtsam::symbol('X', 1);
    gtsam::Key                               X2 = gtsam::symbol('X', 2);
    gtsam::Key                               X3 = gtsam::symbol('X', 3);
    gtsam::Pose3                             p1(gtsam::Rot3::Identity(), gtsam::Point3::Zero());
    gtsam::Pose3                             p2(gtsam::Rot3::Identity(), gtsam::Point3(1, 0, 0));
    gtsam::Pose3                             p3(gtsam::Rot3::Identity(), gtsam::Point3(2, 0, 0));
    gtsam::Pose3                             measure(gtsam::Rot3::Identity(), gtsam::Point3(1, 0, 0));
    gtsam::noiseModel::Isotropic::shared_ptr noise_mode = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);

    gtsam::PriorFactor<gtsam::Pose3>   prior_factor(X1, p1, noise_mode);
    gtsam::PriorFactor<gtsam::Pose3>   prior_factor_x2(X2, p2, noise_mode);
    gtsam::BetweenFactor<gtsam::Pose3> between_factor_12(X1, X2, measure, noise_mode);
    gtsam::BetweenFactor<gtsam::Pose3> between_factor_23(X2, X3, measure, noise_mode);

    gtsam::Values values;
    values.insert(X1, p1);
    values.insert(X2, p2);
    values.insert(X3, p3);

    gtsam::NonlinearFactorGraph nonlinear_factor_graph;
    nonlinear_factor_graph.add(prior_factor);
    nonlinear_factor_graph.add(prior_factor_x2);
    nonlinear_factor_graph.add(between_factor_12);
    nonlinear_factor_graph.add(between_factor_23);
    nonlinear_factor_graph.print("nonlinear_factor_graph : ");

    boost::shared_ptr<gtsam::GaussianFactorGraph> gaussian_factor_graph = nonlinear_factor_graph.linearize(values);
    gaussian_factor_graph->print("lineared_gaussian_factor_graph : ");

    gtsam::VariableIndex variable_index;
    variable_index.augment(*gaussian_factor_graph);
    variable_index.print("augmented variable index : ");

    gtsam::VariableIndex affectedFactorsVarIndex(*gaussian_factor_graph);
    affectedFactorsVarIndex.print("Affected var Factors index : ");
    gtsam::FastMap<gtsam::Key, int> constraintGroups;
    const gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrained(affectedFactorsVarIndex, constraintGroups);
    ordering.print("colamd ordering : ");

    gtsam::GaussianEliminationTree etree(*gaussian_factor_graph, affectedFactorsVarIndex, ordering);
    etree.print("Gaussian Eliminate treee : ");
    std::cout << std::endl;

    gtsam::ISAM2Params isam2_params;
    isam2_params.print("ISAM2 params : ");
    const auto eliminate_func = isam2_params.getEliminationFunction();
    std::cout << std::endl;

    gtsam::ISAM2UpdateParams update_params;
    gtsam::UpdateImpl        update(isam2_params, update_params);
    // update.linearizeNewFactors(nonlinear_factor_graph,values, nonlinear_factor_graph.size(), ,nullptr);

    gtsam::ISAM2JunctionTree junction_tree(etree);
    junction_tree.print("junction tree : ");
    std::cout << std::endl;

    const boost::shared_ptr<gtsam::ISAM2BayesTree> isam2_bayes_tree = junction_tree.eliminate(eliminate_func).first;
    isam2_bayes_tree->print("isam2_bayes_tree : ");
    std::cout << std::endl;
    nonlinear_factor_graph.saveGraph("./1.viz");
    std::cout << nonlinear_factor_graph.dot() << std::endl;

    return 0;
}
