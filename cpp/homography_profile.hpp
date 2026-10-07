#ifndef OPENCV_CALIB3D_HOMOGRAPHY_PROFILE_HPP
#define OPENCV_CALIB3D_HOMOGRAPHY_PROFILE_HPP

#include "opencv_calib3d_shim.h"
#include <cmath>

namespace opencv_calib3d_detail {
// No normalization assumption, epsilon rejection, clamping, or squared norm.
// Check individual products too: nonfinite intermediates are never inliers.
inline bool map_homography(const opencv_calib3d_homography &h,
                           double x, double y, double &xp, double &yp) noexcept {
    const double products[] = {h.h00*x, h.h01*y, h.h10*x, h.h11*y,
                               h.h20*x, h.h21*y};
    for (double value : products) if (!std::isfinite(value)) return false;
    const double nx = products[0] + products[1] + h.h02;
    const double ny = products[2] + products[3] + h.h12;
    const double w = products[4] + products[5] + h.h22;
    if (!std::isfinite(nx) || !std::isfinite(ny) || !std::isfinite(w) || w == 0)
        return false;
    xp = nx / w;
    yp = ny / w;
    return std::isfinite(xp) && std::isfinite(yp);
}

inline bool final_homography_inlier(const opencv_calib3d_homography &h,
                                    double x, double y, double dx, double dy,
                                    double threshold) noexcept {
    double xp = 0, yp = 0;
    if (!map_homography(h, x, y, xp, yp)) return false;
    const double error = std::hypot(xp - dx, yp - dy);
    return std::isfinite(error) && error <= threshold;
}
} // namespace opencv_calib3d_detail
#endif