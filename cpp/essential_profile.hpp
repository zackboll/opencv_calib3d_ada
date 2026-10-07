#ifndef OPENCV_CALIB3D_ESSENTIAL_PROFILE_HPP
#define OPENCV_CALIB3D_ESSENTIAL_PROFILE_HPP

#include "opencv_calib3d_shim.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

namespace opencv_calib3d_detail {
static_assert(std::numeric_limits<double>::is_iec559 && DBL_MANT_DIG == 53 &&
              DBL_MAX_EXP == 1024, "essential profile requires IEEE binary64");
static_assert(std::numeric_limits<float>::is_iec559 && FLT_MANT_DIG == 24 &&
              FLT_MAX_EXP == 128, "essential profile requires IEEE binary32");

inline bool essential_options_fit(double threshold, double confidence) noexcept {
    if (!std::isfinite(threshold) || threshold <= 0 ||
        !std::isfinite(confidence) || confidence <= 0 || confidence >= 1) return false;
    const double squared = threshold * threshold;
    const double overflow_boundary = static_cast<double>(FLT_MAX) + 0x1p103;
    if (!std::isfinite(squared) || squared >= overflow_boundary) return false;
    const float native_squared = squared > static_cast<double>(FLT_MAX)
        ? FLT_MAX : static_cast<float>(squared);
    return std::isfinite(native_squared) && native_squared > 0;
}

inline bool essential_finite(const opencv_calib3d_essential &e,
                             double x1, double y1, double x2, double y2) noexcept {
    for (double v : {e.e00,e.e01,e.e02,e.e10,e.e11,e.e12,e.e20,e.e21,e.e22,
                     x1,y1,x2,y2}) if (!std::isfinite(v)) return false;
    return true;
}

inline bool essential_sum3(double a, double x, double b, double y, double c,
                          double &value) noexcept {
    const double ax = a*x, by = b*y;
    if (!std::isfinite(ax) || !std::isfinite(by)) return false;
    const double sum = ax + by;
    if (!std::isfinite(sum)) return false;
    value = sum + c;
    return std::isfinite(value);
}

// Square root of the native Sampson approximation, in normalized units.
// No arbitrary squares, denominator epsilon, clamping, or NaN publication.
inline bool normalized_sampson_error(const opencv_calib3d_essential &e,
                                     double x1, double y1, double x2, double y2,
                                     double &error) noexcept {
    error = 0;
    if (!essential_finite(e,x1,y1,x2,y2)) return false;
    double ax,ay,az,bx,by,bz,residual;
    if (!essential_sum3(e.e00,x1,e.e01,y1,e.e02,ax) ||
        !essential_sum3(e.e10,x1,e.e11,y1,e.e12,ay) ||
        !essential_sum3(e.e20,x1,e.e21,y1,e.e22,az) ||
        !essential_sum3(e.e00,x2,e.e10,y2,e.e20,bx) ||
        !essential_sum3(e.e01,x2,e.e11,y2,e.e21,by) ||
        !essential_sum3(e.e02,x2,e.e12,y2,e.e22,bz) ||
        !essential_sum3(ax,x2,ay,y2,az,residual)) return false;
    const double norm = std::hypot(std::hypot(ax,ay),std::hypot(bx,by));
    if (!std::isfinite(norm) || norm == 0) return false;
    const double value = std::abs(residual)/norm;
    if (!std::isfinite(value)) return false;
    error = value;
    return true;
}

inline bool final_essential_inlier(const opencv_calib3d_essential &e,
                                   double x1, double y1, double x2, double y2,
                                   double threshold) noexcept {
    double error = 0;
    return normalized_sampson_error(e,x1,y1,x2,y2,error) && error <= threshold;
}
} // namespace opencv_calib3d_detail
#endif