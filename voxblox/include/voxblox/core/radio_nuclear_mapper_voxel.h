/// Plagiarism Notice:
/// The code in this file comes from file voxel.h (in particular from structure IntensityVoxel)
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited lines of code are marked with the comment "RH"

#ifndef VOXBLOX_CORE_RADIO_NUCLEAR_MAPPER_VOXEL_H /// RH
#define VOXBLOX_CORE_RADIO_NUCLEAR_MAPPER_VOXEL_H /// RH

#include <cstdint>
#include <string>

#include "voxblox/core/color.h"
#include "voxblox/core/common.h"

namespace voxblox {
  struct RadiationVoxel { /// RH
    float intensity = 0.0f;
    float distance = std::numeric_limits<float>::infinity(); /// RH
  };
}

#endif //VOXBLOX_CORE_RADIO_NUCLEAR_MAPPER_VOXEL_H /// RH