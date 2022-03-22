/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridFactor.cpp
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#include <gtsam/hybrid/HybridFactor.h>

namespace gtsam {

KeyVector CollectKeys(const KeyVector &continuousKeys,
                      const DiscreteKeys &discreteKeys) {
  KeyVector allKeys;
  std::copy(continuousKeys.begin(), continuousKeys.end(),
            std::back_inserter(allKeys));
  std::transform(discreteKeys.begin(), discreteKeys.end(),
                 std::back_inserter(allKeys),
                 [](const DiscreteKey &k) { return k.first; });
  return allKeys;
}

KeyVector CollectKeys(const KeyVector &keys1, const KeyVector &keys2) {
  KeyVector allKeys;
  std::copy(keys1.begin(), keys1.end(), std::back_inserter(allKeys));
  std::copy(keys2.begin(), keys2.end(), std::back_inserter(allKeys));
  return allKeys;
}

DiscreteKeys CollectDiscreteKeys(const DiscreteKeys &key1,
                                 const DiscreteKeys &key2) {
  DiscreteKeys allKeys;
  std::copy(key1.begin(), key1.end(), std::back_inserter(allKeys));
  std::copy(key2.begin(), key2.end(), std::back_inserter(allKeys));
  return allKeys;
}

HybridFactor::HybridFactor() = default;

HybridFactor::HybridFactor(const KeyVector &keys)
    : Base(keys), isContinuous_(true) {}

HybridFactor::HybridFactor(const KeyVector &continuousKeys,
                           const DiscreteKeys &discreteKeys)
    : Base(CollectKeys(continuousKeys, discreteKeys)),
      isDiscrete_((continuousKeys.size() == 0) && (discreteKeys.size() != 0)),
      isContinuous_((continuousKeys.size() != 0) && (discreteKeys.size() == 0)),
      isHybrid_((continuousKeys.size() != 0) && (discreteKeys.size() != 0)),
      discreteKeys_(discreteKeys) {}

HybridFactor::HybridFactor(const DiscreteKeys &discreteKeys)
    : Base(CollectKeys({}, discreteKeys)),
      isDiscrete_(true),
      discreteKeys_(discreteKeys) {}

void HybridFactor::print(
      const std::string &s,
      const KeyFormatter &formatter) const {
  std::cout << s;
  if (isContinuous_) std::cout << "Cont. ";
  if (isDiscrete_) std::cout << "Disc. ";
  if (isHybrid_) std::cout << "Hybr. ";
  this->printKeys("", formatter);
}

HybridFactor::~HybridFactor() = default;

}  // namespace gtsam