#include <istream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

class Pose3BetweenFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>
{
private:
    gtsam::Pose3 measured_;

public:
    typedef typename gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3> Base;
    Pose3BetweenFactor(gtsam::Key key1, gtsam::Key key2, const gtsam::Pose3& measured,
                       const gtsam::SharedNoiseModel& mode = nullptr)
      : Base(mode, key1, key2), measured_(measured)
    {
    }

    ~Pose3BetweenFactor() override
    {
    }

    /// print with optional string
    void print(const std::string&         s            = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "Pose3BetweenFactor(" << keyFormatter(this->key1()) << "," << keyFormatter(this->key2())
                  << ")\n";
        gtsam::traits<gtsam::Pose3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /// evaluate error, returns vector of errors size of tangent space
    gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(p1, p2, H1, H2);

        typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlocal;
        gtsam::Vector rval = gtsam::traits<gtsam::Pose3>::Local(measured_, hx, boost::none, (H1 || H2) ? &Hlocal : 0);

        if (H1)
            *H1 = Hlocal * (*H1);
        if (H2)
            *H2 = Hlocal * (*H2);
        return rval;
    }
};

class Pose3PriorFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
private:
    gtsam::Pose3 measure_prior_;

public:
    typedef typename gtsam::NoiseModelFactorN<gtsam::Pose3> Base;
    Pose3PriorFactor(gtsam::Key key, const gtsam::Pose3& prior, const gtsam::SharedNoiseModel& mode = nullptr)
      : Base(mode, key), measure_prior_(prior)
    {
    }

    ~Pose3PriorFactor() override
    {
    }

    /// print with optional string
    void print(const std::string&         s            = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "Pose3PriorFactor(" << keyFormatter(this->key1()) << ")\n";
        gtsam::traits<gtsam::Pose3>::Print(measure_prior_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /// evaluate error, returns vector of errors size of tangent space
    gtsam::Vector evaluateError(const gtsam::Pose3&             pose,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        gtsam::Vector rval = gtsam::traits<gtsam::Pose3>::Local(pose, measure_prior_, H, boost::none);
        return rval;
    }
};

int main()
{
    gtsam::Symbol X1('X', 1);
    gtsam::Symbol X2('X', 2);
    gtsam::Pose3  wTb1(gtsam::Rot3::AxisAngle(gtsam::Unit3(0, 0, 1), M_PI_4 + 0.01), gtsam::Point3(1, 0, 0));
    gtsam::Pose3  wTb2(gtsam::Rot3::AxisAngle(gtsam::Unit3(0, 0, 1), M_PI_2 + 0.02), gtsam::Point3(1, 0, 0));

    // initial values
    gtsam::Values values;
    values.insert<gtsam::Pose3>(X1, wTb1);
    values.insert<gtsam::Pose3>(X2, wTb2);
    values.print("all values : ");

    // measurement
    gtsam::Pose3::TangentVector noise;
    noise << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    gtsam::Pose3 measure = wTb1.between(wTb2).expmap(noise);

    // pose3 noise model
    gtsam::noiseModel::Isotropic::shared_ptr noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);

    // between factor
    Pose3BetweenFactor self_pose3_between_factor(X1, X2, measure, noise_model);

    // prior factor
    Pose3PriorFactor                 self_prior_factor(X1, wTb1.expmap(noise), noise_model);
    gtsam::PriorFactor<gtsam::Pose3> sys_prior_factor(X1, wTb1.expmap(noise), noise_model);

    // factor graph with prior factor
    gtsam::NonlinearFactorGraph factor_graph;
    factor_graph.add(self_pose3_between_factor);
    factor_graph.add(self_prior_factor);
    // factor_graph.add(sys_prior_factor);

    // construct isam
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;
    parameters.print("isam2 parameters : ");
    gtsam::ISAM2 isam(parameters);

    // isam2 update
    isam.update(factor_graph, values);
    isam.update();
    isam.update();

    // get isam2 estimate values
    gtsam::Values est_values = isam.calculateEstimate();

    est_values.print("est : ");
    isam.saveGraph("./self_test_between_factor.dot");
}
