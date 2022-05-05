#include "kkt_subsystem.h"
#include "conex/debug_macros.h"
#define CONEX_ENABLE_TIMER 1

namespace conex {
using T = ConvexSetNode;

T::ConvexSetNode(const std::vector<int>& variables, 
                 const ConvexSetNodeParameters& params) : 
                 KKTSubsystem(variables, 0), params_(params) {
  int num_supernodes =
      params.num_incoming * (2 * params.spatial_dimension + 1) +
      params.spatial_dimension + 1;

  std::vector<int> supernodes;
  supernodes.insert(supernodes.begin(), variables.begin(),
                    variables.begin() + num_supernodes);

  std::vector<int> separators;
  separators.insert(separators.begin(), variables.begin() + num_supernodes,
                    variables.end());
  SetSupernodes(supernodes);
  SetSeparators(separators);
  num_incoming = params.num_incoming; 
  num_outgoing = params.num_outgoing;
  spatial_dim = params.spatial_dimension;
}

} // namespace conex
