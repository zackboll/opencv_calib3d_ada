#include "correction_profile.hpp"
#include <cassert>
#include <iostream>

int main() {
    using namespace opencv_calib3d_detail;
    const opencv_calib3d_correction_options good{1e-3, 1e-9, 1e-6, 1e-6, 1e-9, 1e-9};
    assert(correction_options_fit(good));
    // Inclusive/exclusive range boundaries of each documented option.
    auto with = [&](int field, double value) {
        auto o = good;
        double *f[] = {&o.minimum_second_singular_ratio, &o.maximum_third_singular_ratio,
                       &o.minimum_epipolar_line_ratio, &o.maximum_corrected_error_pixels,
                       &o.absolute_displacement_excess_pixels, &o.relative_displacement_excess};
        *f[field] = value;
        return correction_options_fit(o);
    };
    for (int field = 0; field < 6; ++field)
        for (double bad : {NAN * 1., INFINITY * 1., -INFINITY * 1., -1e-300})
            assert(!with(field, bad));
    assert(with(0, 1) && !with(0, 0) && !with(0, std::nextafter(1., 2.)));
    assert(with(1, 0) && !with(1, 1) && !with(1, 1e-3));          // must stay < minimum second
    { auto o = good; o.minimum_second_singular_ratio = 1; o.maximum_third_singular_ratio =
          std::nextafter(1., 0.); assert(correction_options_fit(o)); }
    assert(with(2, 0) && with(2, 1) && !with(2, std::nextafter(1., 2.)));
    assert(with(3, 0) && with(3, 1e300) && with(4, 0) && with(4, 1e300));
    assert(with(5, 0) && with(5, 1) && !with(5, std::nextafter(1., 2.)));

    // Normalization: sign, scale and zero pattern preserved; extreme scales exact.
    double n[9];
    for (double s : {1., -1., 2., .5, 1e-100, 1e-40, 1e40, 1e100, 1e-300, 1e300}) {
        const opencv_calib3d_fundamental f{0, 0, 0, 0, 0, -s, 0, s, 0};
        assert(normalize_fundamental(f, n) == fundamental_normalization::ok);
        assert(n[5] == (s > 0 ? -1 : 1) && n[7] == (s > 0 ? 1 : -1) && n[0] == 0 && n[8] == 0);
    }
    assert(normalize_fundamental({}, n) == fundamental_normalization::zero);
    assert(normalize_fundamental({NAN, 0, 0, 0, 0, 1, 0, -1, 0}, n) == fundamental_normalization::nonfinite);
    assert(normalize_fundamental({0, 0, 0, 0, 0, INFINITY, 0, -1, 0}, n) == fundamental_normalization::nonfinite);
    // Coefficient range that cannot be represented after normalization is rejected.
    assert(normalize_fundamental({0, 0, 0, 0, 0, -1e300, 0, 1e300, 4e-300}, n) ==
           fundamental_normalization::unrepresentable);
    assert(normalize_fundamental({0, 0, 0, 0, 0, -1, 0, 1, 1e-310}, n) ==
           fundamental_normalization::unrepresentable);
    assert(normalize_fundamental({0, 0, 0, 0, 0, -1, 0, 1, DBL_MIN}, n) == fundamental_normalization::ok);

    // Rectified oracle: l2=(0,-1,y1), l1=(0,1,-y2); d1=|y1-y2|, d2=|y2-y1|.
    const double rect[9] = {0, 0, 0, 0, 0, -1, 0, 1, 0};
    double n1, n2, d1, d2, q;
    assert(epipolar_distances(rect, 100, 20, 90, 24, n1, n2, d1, d2) == epipolar_geometry::ok);
    assert(n1 == 1 && n2 == 1 && d1 == 4 && d2 == 4);
    assert(epipolar_line_quality(rect, 100, 20, 90, 24, n1, n2, q));
    const double expected = std::min(1 / (std::sqrt(2.) * std::hypot(100., 20., 1.)),
                                     1 / (std::sqrt(2.) * std::hypot(90., 24., 1.)));
    assert(std::abs(q - expected) < 1e-15 * expected);
    // Quality invariance to common F scale (computed on the normalized F anyway).
    const double rect7[9] = {0, 0, 0, 0, 0, -7, 0, 7, 0};
    double q7;
    assert(epipolar_distances(rect7, 100, 20, 90, 24, n1, n2, d1, d2) == epipolar_geometry::ok);
    assert(epipolar_line_quality(rect7, 100, 20, 90, 24, n1, n2, q7) && std::abs(q7 - q) < 1e-15 * q);
    // Exactly at an epipole: undefined line, never an epsilon.
    const double ex[9] = {0, -1, 50, 1, 0, -100, -50, 100, 0};
    assert(epipolar_distances(ex, 100, 50, 130, 75, n1, n2, d1, d2) == epipolar_geometry::undefined_line);
    assert(epipolar_distances(ex, 100 + 1e-9, 50, 130, 75, n1, n2, d1, d2) == epipolar_geometry::ok);
    assert(epipolar_distances(rect, 1e308, 1e308, 1, 1, n1, n2, d1, d2) == epipolar_geometry::ok ||
           epipolar_distances(rect, 1e308, 1e308, 1, 1, n1, n2, d1, d2) == epipolar_geometry::unrepresentable);
    assert(epipolar_distances(ex, 1e308, 1e308, 1, 1, n1, n2, d1, d2) == epipolar_geometry::unrepresentable);
    assert(epipolar_distances(rect, NAN, 1, 1, 1, n1, n2, d1, d2) == epipolar_geometry::unrepresentable);

    // Scaled four-dimensional norm: sqrt(8) oracle and no overflow from squaring.
    double value;
    assert(displacement_norm4({0, 2, 0, -2}, value) && std::abs(value - std::sqrt(8.)) < 1e-15);
    assert(displacement_norm4({1e300, 1e300, 0, 0}, value) && std::isfinite(value) &&
           std::abs(value / 1e300 - std::sqrt(2.)) < 1e-15);
    assert(displacement_norm4({0, 0, 0, 0}, value) && value == 0);
    assert(!displacement_norm4({INFINITY, 0, 0, 0}, value));
    // Allowance: overflow is an error, never a silent pass.
    double allowed;
    assert(displacement_allowance(4, 0, 0, allowed) && allowed == 4);
    assert(displacement_allowance(4, 1e-9, 1e-12, allowed) && allowed > 4);
    assert(!displacement_allowance(DBL_MAX, 1, 0, allowed));
    assert(!displacement_allowance(DBL_MAX, 0, DBL_MAX, allowed));
    std::cout << "PASS: correction options ranges, F normalization/underflow, rectified line "
                 "oracle, scale-invariant quality, exact epipole, scaled norm and allowance\n";
}
