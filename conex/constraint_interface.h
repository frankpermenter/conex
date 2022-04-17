#pragma once
#include "conex/visitor.h"

namespace conex {
class ConstraintBase {
 public:
  virtual void accept(Visitor*) = 0;
};
}
