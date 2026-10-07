#ifndef OPENCV_CALIB3D_FUNDAMENTAL_PROFILE_HPP
#define OPENCV_CALIB3D_FUNDAMENTAL_PROFILE_HPP

#include "opencv_calib3d_shim.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

namespace opencv_calib3d_detail {
static_assert(std::numeric_limits<double>::is_iec559 && DBL_MANT_DIG == 53 &&
              DBL_MAX_EXP == 1024 && DBL_EPSILON == 0x1p-52,
              "fundamental profile requires IEEE binary64");
static_assert(std::numeric_limits<float>::is_iec559 && FLT_MANT_DIG == 24 &&
              FLT_MAX_EXP == 128, "fundamental profile requires IEEE binary32");

inline bool fundamental_options_fit(double threshold, double confidence) noexcept {
    if (!std::isfinite(threshold) || threshold <= 0 ||
        !std::isfinite(confidence) || confidence < DBL_EPSILON ||
        confidence > 1 - DBL_EPSILON) return false;
    // Native findInliers squares in double, THEN converts to float. Avoid an
    // out-of-range language conversion; preserve rounding near FLT_MAX.
    const double squared = threshold * threshold;
    const double overflow_boundary = static_cast<double>(FLT_MAX) + 0x1p103;
    if (!std::isfinite(squared) || squared >= overflow_boundary) return false;
    // Values above FLT_MAX but below its rounding-overflow midpoint round to
    // FLT_MAX; avoid the C++ out-of-range conversion in that small interval.
    const float native_squared = squared > static_cast<double>(FLT_MAX)
        ? FLT_MAX : static_cast<float>(squared);
    return std::isfinite(native_squared) && native_squared > 0;
}

inline bool finite_sum3(double a, double x, double b, double y, double c,
                        double &value) noexcept {
    const double ax = a*x, by = b*y;
    if (!std::isfinite(ax) || !std::isfinite(by)) return false;
    const double sum = ax + by;
    if (!std::isfinite(sum)) return false;
    value = sum + c;
    return std::isfinite(value);
}

// Maximum of two point-to-epipolar-line distances, NOT Sampson distance.
// Undefined geometry is never an inlier; no epsilon, clamping, or squared norm.
inline bool maximum_epipolar_error(const opencv_calib3d_fundamental &f,
                                   double x1, double y1, double x2, double y2,
                                   double &error) noexcept {
    error = 0;
    for (double v : {f.f00,f.f01,f.f02,f.f10,f.f11,f.f12,f.f20,f.f21,f.f22,
                     x1,y1,x2,y2}) if (!std::isfinite(v)) return false;
    double a2,b2,c2,a1,b1,c1,n1,n2;
    if (!finite_sum3(f.f00,x1,f.f01,y1,f.f02,a2) ||
        !finite_sum3(f.f10,x1,f.f11,y1,f.f12,b2) ||
        !finite_sum3(f.f20,x1,f.f21,y1,f.f22,c2) ||
        !finite_sum3(f.f00,x2,f.f10,y2,f.f20,a1) ||
        !finite_sum3(f.f01,x2,f.f11,y2,f.f21,b1) ||
        !finite_sum3(f.f02,x2,f.f12,y2,f.f22,c1) ||
        !finite_sum3(a1,x1,b1,y1,c1,n1) ||
        !finite_sum3(a2,x2,b2,y2,c2,n2)) return false;
    const double norm1 = std::hypot(a1,b1), norm2 = std::hypot(a2,b2);
    if (!std::isfinite(norm1) || !std::isfinite(norm2) || norm1 == 0 || norm2 == 0)
        return false;
    const double d1 = std::abs(n1)/norm1, d2 = std::abs(n2)/norm2;
    if (!std::isfinite(d1) || !std::isfinite(d2)) return false;
    error = std::max(d1,d2);
    return true;
}

inline bool final_fundamental_inlier(const opencv_calib3d_fundamental &f,
                                     double x1, double y1, double x2, double y2,
                                     double threshold) noexcept {
    double error = 0;
    return maximum_epipolar_error(f,x1,y1,x2,y2,error) && error <= threshold;
}
} // namespace opencv_calib3d_detail
#endif
