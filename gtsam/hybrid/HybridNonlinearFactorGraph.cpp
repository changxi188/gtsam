/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactorGraph.cpp
 * @brief  Nonlinear hybrid factor graph that uses type erasure
 * @author Varun Agrawal
 * @date   May 28, 2022
 */

#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>

namespace gtsam {

/* ************************************************************************* */
void HybridNonlinearFactorGraph::add(
    boost::shared_ptr<NonlinearFactor> factor) {
  FactorGraph::add(boost::make_shared<HybridNonlinearFactor>(factor));
}

/* ************************************************************************* */
void HybridNonlinearFactorGraph::add(boost::shared_ptr<DiscreteFactor> factor) {
  FactorGraph::add(boost::make_shared<HybridDiscreteFactor>(factor));
}

/* ************************************************************************* */
void HybridNonlinearFactorGraph::print(const std::string& s,
                                       const KeyFormatter& keyFormatter) const {
  // Base::print(str, keyFormatter);
  std::cout << (s.empty() ? "" : s + " ") << std::endl;
  std::cout << "size: " << size() << std::endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    std::stringstream ss;
    ss << "factor " << i << ": ";
    if (factors_[i]) {
      factors_[i]->print(ss.str(), keyFormatter);
      std::cout << std::endl;
    }
  }
}

/* ************************************************************************* */
HybridGaussianFactorGraph::shared_ptr HybridNonlinearFactorGraph::linearize(
    const Values& continuousValues) const {
  using boost::dynamic_pointer_cast;

  // create an empty linear FG
  auto linearFG = boost::make_shared<HybridGaussianFactorGraph>();

  linearFG->reserve(size());

  // linearize all hybrid factors
  for (auto& f : factors_) {
    // First check if it is a valid factor
    if (!f) {
      // TODO(dellaert): why?
      linearFG->push_back(GaussianFactor::shared_ptr());
      continue;
    }
    // Check if it is a nonlinear mixture factor
    if (auto nlmf = dynamic_pointer_cast<MixtureFactor>(f)) {
      const GaussianMixtureFactor::shared_ptr& gmf =
          nlmf->linearize(continuousValues);
      linearFG->push_back(gmf);
    } else if (auto nlhf = dynamic_pointer_cast<HybridNonlinearFactor>(f)) {
      // Nonlinear wrapper case:
      const GaussianFactor::shared_ptr& gf =
          nlhf->inner()->linearize(continuousValues);
      const auto hgf = boost::make_shared<HybridGaussianFactor>(gf);
      linearFG->push_back(hgf);
    } else if (dynamic_pointer_cast<DiscreteFactor>(f) ||
               dynamic_pointer_cast<HybridDiscreteFactor>(f)) {
      // If discrete-only: doesn't need linearization.
      linearFG->push_back(f);
    } else {
      auto& fr = *f;
      throw std::invalid_argument(
          std::string("HybridNonlinearFactorGraph::linearize: factor type "
                      "not handled: ") +
          demangle(typeid(fr).name()));
    }
  }
  return linearFG;
}

}  // namespace gtsam
