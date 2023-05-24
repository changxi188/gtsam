#include <istream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>

int main()
{
    gtsam::Symbol                      x1('x', 1);
    gtsam::Symbol                      x2('x', 2);
    gtsam::Pose3                       pose1;
    gtsam::Pose3                       pose2(gtsam::Rot3::Rodrigues(0, 0, 1.6), gtsam::Point3(1, 2, 3));
    gtsam::Pose3                       delta      = pose1.between(pose2);
    gtsam::SharedDiagonal              noise_mode = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(x1, x2, delta, noise_mode);

    std::cout << "measured : " << between_factor.measured() << std::endl;
}
