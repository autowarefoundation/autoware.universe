#pragma once
#include <modularized_particle_filter/correction/abst_corrector.hpp>

namespace particle_filter
{
class CameraParticleCorrector : public AbstCorrector
{
public:
  CameraParticleCorrector() : AbstCorrector("camera_particle_corrector") {}

private:
};
}  // namespace particle_filter