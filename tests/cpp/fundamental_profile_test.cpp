#include "fundamental_profile.hpp"
#include <cassert>
#include <iostream>

int main() {
    using namespace opencv_calib3d_detail;
    const double epsilon = DBL_EPSILON;
    assert(fundamental_options_fit(3,epsilon));
    assert(fundamental_options_fit(3,1-epsilon));
    assert(!fundamental_options_fit(3,std::nextafter(epsilon,0)));
    assert(!fundamental_options_fit(3,std::nextafter(1-epsilon,1)));
    for (double t : {0.,-1.,1e-30,1e20,INFINITY*1.,NAN*1.})
        assert(!fundamental_options_fit(t,.99));
    assert(static_cast<float>(1e-30) > 0); // conversion-before-square is WRONG
    assert(fundamental_options_fit(1e-22,.99));
    const double underflow = std::sqrt(0x1p-150);
    assert(!fundamental_options_fit(underflow,.99));
    assert(fundamental_options_fit(std::nextafter(underflow,INFINITY),.99));
    const double overflow = std::sqrt(static_cast<double>(FLT_MAX)+0x1p103);
    assert(!fundamental_options_fit(overflow,.99));
    assert(fundamental_options_fit(std::nextafter(overflow,0),.99));
    for (double scale : {1.,-7.,1e6,1e200}) {
        opencv_calib3d_fundamental f{0,0,0,0,0,scale,0,-scale,0};
        double error = -1;
        assert(maximum_epipolar_error(f,100,50,80,50,error) && error == 0);
        assert(maximum_epipolar_error(f,100,50,80,53,error) && std::abs(error-3)<1e-12);
        assert(final_fundamental_inlier(f,100,50,80,53,3.00000000001));
        assert(!final_fundamental_inlier(f,100,50,80,53,2));
    }
    double error = -1;
    assert(!maximum_epipolar_error({},100,50,80,53,error));
    opencv_calib3d_fundamental f{DBL_MAX,0,0,0,0,1,0,-1,0};
    assert(!maximum_epipolar_error(f,2,50,80,53,error));
    std::cout << "PASS: fundamental binary64/binary32 squared-threshold rounding boundaries, "
                 "inclusive confidence, independent 0/3-pixel oracle, scale/range/undefined\n";
}
