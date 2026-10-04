#ifndef OPENCV_CALIB3D_PNP_PROFILE_HPP
#define OPENCV_CALIB3D_PNP_PROFILE_HPP

#include <cmath>
#include <cstdint>
#include <limits>

namespace opencv_calib3d_detail {
inline bool finite(double value) noexcept { return std::isfinite(value); }
inline bool intrinsics_fit(double fx, double fy, double cx, double cy) noexcept {
    return finite(fx) && finite(fy) && finite(cx) && finite(cy) && fx > 0.0 && fy > 0.0;
}
inline bool distortion5_fit(double k1, double k2, double p1, double p2, double k3) noexcept {
    return finite(k1) && finite(k2) && finite(p1) && finite(p2) && finite(k3);
}
inline bool ransac_options_fit(std::int64_t iterations, double reprojection_error,
                               double confidence) noexcept {
    return iterations > 0 && iterations <= std::numeric_limits<std::int32_t>::max() &&
           finite(reprojection_error) && reprojection_error > 0.0 &&
           reprojection_error <= std::numeric_limits<float>::max() &&
            static_cast<float>(reprojection_error) > 0.0F &&
           finite(confidence) && confidence > 0.0 && confidence < 1.0;
}
} // namespace opencv_calib3d_detail

#endif
