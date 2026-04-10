// CBF (Conic Benchmark Format) reader: converts conic problems to Problem.
//
// Supports LP, SOCP, and SDP constraints from cblib.zib.de.
//
// CBF format sections:
//   VER        version
//   OBJSENSE   MIN or MAX
//   VAR        number of variables, cone specifications
//   CON        number of constraints, cone specifications
//   OBJACOORD  objective sparse entries
//   ACOORD     constraint matrix sparse entries
//   BCOORD     constraint RHS entries
//   PSDVAR     PSD variable blocks (for SDP)
//   PSDCON     PSD constraint blocks
//   HCOORD     PSD constraint entries
//   DCOORD     PSD objective entries
//
// Cone types: F (free), L+ (nonneg), L- (nonpos), Q (SOC), QR (rotated SOC)

#pragma once
#include <string>
#include "conex/common/problem.h"

namespace conex {

struct CBFInfo {
  std::string objsense;  // "MIN" or "MAX"
  int num_variables = 0;
  int num_constraints = 0;
  int num_psd_vars = 0;
  int num_psd_cons = 0;
};

std::pair<Problem, CBFInfo> ReadCBF(const std::string& filename);

}  // namespace conex
