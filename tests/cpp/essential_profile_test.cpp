#include "essential_profile.hpp"
#include <cassert>
#include <iostream>

int main() {
    using namespace opencv_calib3d_detail;
    const double nan=std::numeric_limits<double>::quiet_NaN();
    assert(essential_options_fit(1e-3,0.999));
    assert(essential_options_fit(1e-3,DBL_EPSILON/2));
    assert(essential_options_fit(1e-3,1-DBL_EPSILON/2));
    for (double t : {0.,-1.,nan,INFINITY*1.,1e-30,1e20,DBL_MAX})
        assert(!essential_options_fit(t,0.999));
    for (double c : {0.,-1.,1.,2.,nan,INFINITY*1.})
        assert(!essential_options_fit(1e-3,c));
    const double underflow=std::sqrt(0x1p-150);
    assert(!essential_options_fit(underflow,.5));
    assert(essential_options_fit(std::nextafter(underflow,INFINITY),.5));
    const double midpoint=static_cast<double>(FLT_MAX)+0x1p103;
    assert(essential_options_fit(std::sqrt(std::nextafter(midpoint,0.)),0.5));
    assert(!essential_options_fit(std::sqrt(midpoint),0.5));
    for (double scale : {1.,-7.,1e6}) {
        const opencv_calib3d_essential e{0,0,0,0,0,-scale,0,scale,0};
        double error=99;
        assert(normalized_sampson_error(e,.2,.1,-.1,.1,error) && error==0);
        assert(normalized_sampson_error(e,.2,.1,-.1,.3,error));
        assert(std::abs(error-.2/std::sqrt(2.))<1e-15);
        assert(final_essential_inlier(e,.2,.1,-.1,.3,.15));
        assert(!final_essential_inlier(e,.2,.1,-.1,.3,.1));
    }
    double error=99;
    assert(!normalized_sampson_error({},.2,.1,-.1,.3,error) && error==0);
    assert(!normalized_sampson_error({DBL_MAX,0,0,0,0,0,0,0,0},2,0,0,0,error));
    assert(!normalized_sampson_error({DBL_MAX,DBL_MAX,0,0,0,0,0,0,0},1,1,0,0,error));
    assert(!normalized_sampson_error({0,0,DBL_MAX,0,0,0,0,0,0},0,0,2,0,error));
    assert(!normalized_sampson_error({0,0,1e-300,0,0,0,0,0,DBL_MAX},0,0,0,0,error));
    assert(!normalized_sampson_error({0,0,DBL_MAX,0,0,DBL_MAX,0,0,0},0,0,0,0,error));
    assert(!normalized_sampson_error({0,0,0,0,0,1,0,-1,0},nan,0,0,0,error));
    std::cout << "PASS: essential IEEE threshold/confidence profile and independent Sampson oracle\n";
}