#include <iostream>

#include <gtsam/inference/Factor.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>

int main()
{
    gtsam::VectorValues vector_values;
    gtsam::Key          x1 = gtsam::symbol('x', 1);
    gtsam::Vector3      v3(0, 0, 0);
    vector_values.insert(x1, v3);
    vector_values.print("vector_values");

    boost::shared_ptr<gtsam::Factor>  factor;
    gtsam::FactorGraph<gtsam::Factor> factor_graph({factor});
    factor_graph.print("factor_graph : ");
    factor_graph.saveGraph("./1.viz");
}
