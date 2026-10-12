#include "pnp_profile.hpp"
#include <cassert>
#include <cstdint>
#include <limits>
int main() {
    using namespace opencv_calib3d_detail;
    assert(intrinsics_fit(800.0, 820.0, 320.0, 240.0));
    assert(!intrinsics_fit(0.0, 820.0, 320.0, 240.0));
    assert(!intrinsics_fit(-1.0, 820.0, 320.0, 240.0));
    assert(distortion5_fit(0.0, 0.0, 0.0, 0.0, 0.0));
    assert(ransac_options_fit(100, 8.0, 0.99));
    assert(!ransac_options_fit(0, 8.0, 0.99));
    assert(!ransac_options_fit(100, 0.0, 0.99));
    assert(!ransac_options_fit(100, 1e-300, 0.99));
    assert(!ransac_options_fit(100, 8.0, 0.0));
    assert(!ransac_options_fit(100, 8.0, 1.0));
    assert(!ransac_options_fit(static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) + 1,
                               8.0, 0.99));
    const double world[3][3] = {{0.3, -0.2, 3.5}, {1.4, 0.1, 4.0}, {-0.5, 1.2, 5.3}};
    const double rays[3][2] = {{(700.0 - 640.0) / 800.0, (300.0 - 360.0) / 820.0},
                                {(900.0 - 640.0) / 800.0, (400.0 - 360.0) / 820.0},
                                {(500.0 - 640.0) / 800.0, (550.0 - 360.0) / 820.0}};
    const double repeated[3][3] = {{1, 2, 3}, {1, 2, 3}, {0, 1, 4}};
    const double collinear[3][3] = {{0, 0, 1}, {1, 0, 2}, {2, 0, 3}};
    const double same[3][2] = {{(400.0 - 640.0) / 800.0, (300.0 - 360.0) / 820.0},
                                {(400.0 - 640.0) / 800.0, (300.0 - 360.0) / 820.0},
                                {(500.0 - 640.0) / 800.0, (420.0 - 360.0) / 820.0}};
    const double coplanar[3][2] = {{0.0, 0.0}, {(700.0 - 640.0) / 800.0, 0.0},
                                    {(500.0 - 640.0) / 800.0, 0.0}};
    const double huge[3][3] = {{1e200, 0, 1e200}, {0, 1e200, 1e200}, {-1e200, -1e200, 2e200}};
    const double distorted_pixels[3][2] = {{-0.55, 0.55}, {0.0, 0.525}, {0.55, 0.55}};
    const double recovered[3][2] = {{-0.5, 0.5}, {0.0, 0.5}, {0.5, 0.5}};
    assert(ap3p_world_admissible(world) && ap3p_bearings_admissible(rays));
    assert(!ap3p_world_admissible(repeated));
    assert(!ap3p_world_admissible(collinear));
    assert(!ap3p_bearings_admissible(same));
    assert(!ap3p_bearings_admissible(coplanar));
    assert(!ap3p_world_admissible(huge));
    // Distorted pixels are not the AP3P rays. Their normalized coordinates are
    // admissible; the ideal pre-distortion rays are exactly coplanar.
    assert(ap3p_bearings_admissible(distorted_pixels));
    assert(!ap3p_bearings_admissible(recovered));
    return 0;
}
