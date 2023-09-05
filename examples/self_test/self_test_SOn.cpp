#include <iostream>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/Unit3.h>

int main()
{
    gtsam::Unit3 axis(1, 0, 0);
    double       angle1 = M_PI_4 + 0.02;
    double       angle2 = M_PI_2 + 0.01;
    gtsam::Rot3  r1     = gtsam::Rot3::AxisAngle(axis, angle1);
    gtsam::Rot3  r2     = gtsam::Rot3::AxisAngle(axis, angle2);
    r1.print("rot1 : \n");
    r2.print("rot2 : \n");
    gtsam::Rot3 r_12 = r1.between(r2);
    r_12.print("between r_12 : \n");

    gtsam::Rot3 r22 = gtsam::Rot3::Expmap(axis.unitVector() * angle2);
    // r1.expmap();
    r22.print("rot22 : \n");

    gtsam::Rot3::Jacobian      jacobina1;
    gtsam::Rot3::Jacobian      jacobina2;
    gtsam::Rot3::ChartJacobian optional_jacobian1(jacobina1);
    gtsam::Rot3::ChartJacobian optional_jacobian2(jacobina2);
    r_12 = gtsam::traits<gtsam::Rot3>::Between(r1, r2, optional_jacobian1, optional_jacobian2);

    if (optional_jacobian1)
    {
        std::cout << "jacobina1 : \n" << (*optional_jacobian1) << std::endl;

        r_12.print("between r_12 sec time : \n");
        r2.inverse().compose(r1).print("r2.inverse() * r1 : \n");
    }

    if (optional_jacobian2)
    {
        std::cout << "jacobina2 : \n" << (*optional_jacobian2) << std::endl;
    }

    return 0;
}
