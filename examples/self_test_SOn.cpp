#include <iostream>

#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SOn.h>

int main()
{
    gtsam::Vector3 v_norm = gtsam::Vector3(100, 20, 40).normalized();
    std::cout << "v_norm : " << v_norm << std::endl;
    gtsam::SO3                  so3 = gtsam::SO3::AxisAngle(v_norm, 3.1416);
    gtsam::SO<3>::TangentVector tangent_vector_3d;
    tangent_vector_3d << 1, 2, 3;
    so3.expmap(tangent_vector_3d).print();
    so3.print("so3");
    gtsam::SO<6>                so6;
    gtsam::SO<6>::TangentVector tangent_vector_6d;
    tangent_vector_6d << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
    // so6.expmap(tangent_vector_6d);

    gtsam::Vector3 v3;
    std::cout << "v3 : " << v3.setZero() << std::endl;
    gtsam::SO3 so33;
    so33.print("so33");

    return 0;
}
