#pragma once
#include "validation/overlay.hpp"

namespace validation
{
class RefineOptimizer : public Overlay
{
public:
  RefineOptimizer();

protected:
  void imageCallback(const Image & msg) override;
};
}  // namespace validation