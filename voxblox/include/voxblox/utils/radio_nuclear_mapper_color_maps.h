/// Plagiarism Notice:
/// The code in this file comes from file color_maps.h (in particular from class IronbowColorMap)
/// and has been adapted for the special purpose of radiation mapping by Robert Hülsmann.
/// New/edited lines of code are marked with the comment "RH" (except simple renaming "intensity" to "radiation" etc.).

#ifndef VOXBLOX_UTILS_RADIO_NUCLEAR_MAPPER_COLOR_MAPS_H_
#define VOXBLOX_UTILS_RADIO_NUCLEAR_MAPPER_COLOR_MAPS_H_

#include <algorithm>
#include <vector>

#include "voxblox/core/common.h"
#include "voxblox/core/color.h"

#include "voxblox/utils/color_maps.h" /// RH

namespace voxblox {

  class TrafficLightColorMap : public ColorMap {
  public:
    TrafficLightColorMap() : ColorMap() {
      palette_colors_.push_back(Color(0, 255, 0)); /// green (RH)
      palette_colors_.push_back(Color(255, 255, 0)); /// yellow (RH)
      palette_colors_.push_back(Color(255, 0, 0)); /// red (RH)
      // Add an extra to avoid overflow.
      palette_colors_.push_back(Color(255, 0, 0)); /// red (RH)

      increment_ = 1.0 / (palette_colors_.size() - 2);
    }

    virtual Color colorLookup(float value) const {
      float new_value = std::min(max_value_, std::max(min_value_, value));
      new_value = (new_value - min_value_) / (max_value_ - min_value_);

      size_t index = static_cast<size_t>(std::floor(new_value / increment_));

      return Color::blendTwoColors(
          palette_colors_[index], increment_ * (index + 1) - new_value,
          palette_colors_[index + 1], new_value - increment_ * (index));
    }

  protected:
    std::vector<Color> palette_colors_;
    float increment_;
  };

}  // namespace voxblox

#endif // VOXBLOX_UTILS_RADIO_NUCLEAR_MAPPER_COLOR_MAPS_H_
