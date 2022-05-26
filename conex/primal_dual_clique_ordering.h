#pragma once
#include <vector>

#include "conex/tree_utils.h"

namespace conex {
PrimalDualCliqueTree MakePrimalDualCliqueTree(
    const std::vector<std::vector<int>>& primal_variables,
    const std::vector<std::vector<int>>& dual_variables);
}  // namespace conex
