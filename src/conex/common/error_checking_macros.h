#include <iostream>

#include "conex/common/error_codes.h"

namespace conex {

// TODO(FrankPermenter): return conex error codes on failure.

#define CONEX_RETURN_ON_FAIL(x, msg)                                           \
  if (!(x)) {                                                                  \
    std::cerr << __FILE__ << " line " << __LINE__ << ": " << msg << std::endl; \
    return CONEX_FAILURE;                                                      \
  }

#define CONEX_DEMAND(x, msg)       \
  if (!(x)) {                      \
    throw std::runtime_error(msg); \
  }

#define CONEX_CHECK(x)                                                  \
  if (!(x)) {                                                           \
    std::cerr << __FILE__ << " line " << __LINE__ << ": " << std::endl; \
    throw std::runtime_error(std::string("Condition failed:\n") + #x);  \
  }
#ifdef NDEBUG
#define CONEX_ASSERT(x, msg)  // NOOP
#else
#define CONEX_ASSERT(x, msg)       \
  if (!(x)) {                      \
    throw std::runtime_error(msg); \
  }
#endif
}  // namespace conex
