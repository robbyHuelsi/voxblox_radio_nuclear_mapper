/// Plagiarism Notice:
/// The code in this file comes from file voxel.h (in particular from structure IntensityVoxel)
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited lines of code are marked with the comment "RH" (except simple renaming "intensity" to "radiation" etc.).

#ifndef VOXBLOX_CORE_RADIO_NUCLEAR_MAPPER_VOXEL_H
#define VOXBLOX_CORE_RADIO_NUCLEAR_MAPPER_VOXEL_H

namespace voxblox {
  struct RadiationVoxel {
    bool has_intensity = false;
    float intensity = 0.0f;
    float distance = std::numeric_limits<float>::infinity(); /// RH
  };
}

#endif //VOXBLOX_CORE_RADIO_NUCLEAR_MAPPER_VOXEL_H