#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/linear/JacobianFactor.h>

#include <iostream>

int main()
{
    gtsam::JacobianFactor jacobian_factor;
    // const gtsam::VerticalBlockMatrix matrix_obj = jacobian_factor.matrixObject();
    std::vector<size_t>        dimension{3, 2};
    gtsam::VerticalBlockMatrix matrix_obj(dimension, 5, true);
    std::cout << "row start : " << matrix_obj.rowStart() << std::endl;
    std::cout << "row end : " << matrix_obj.rowEnd() << std::endl;
    std::cout << "block start : " << matrix_obj.firstBlock() << std::endl;
    std::cout << "matrix : " << matrix_obj.matrix() << std::endl;
    std::cout << " ******************************************************************* " << std::endl;
    std::cout << matrix_obj(0) << std::endl;
    std::cout << matrix_obj(1) << std::endl;

    return 0;
}
