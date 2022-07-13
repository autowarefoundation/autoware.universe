/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include "../../example/general/base_alloc.hpp"
# include <cppad/cppad.hpp>

namespace { // BEGIN empty namespace

bool test_parameter(void)
{   bool ok = true;
    typedef CppAD::AD<base_alloc> my_ad;

    // y = x + 0 + 1 + 2 + ... + N-1
    CppAD::vector<my_ad>   a_x, a_y;
    a_x.resize(1);
    a_x[0] = my_ad(1.);
    CppAD::Independent(a_x);
    a_y.resize(1);
    a_y[0] = a_x[0];

    // create a new parameter for each iteration of this loop
    size_t i, N = 50;
    for(i = 0; i < N; i++)
        a_y[0] += double(i);

    CppAD::ADFun<base_alloc> f(a_x, a_y);

    return ok;
}

bool test_numeric_limits(void)
{   bool ok = true;
    typedef CppAD::AD<base_alloc> my_ad;
    //
    base_alloc eps = Value( CppAD::numeric_limits<my_ad>::epsilon() );
    ok            &= *eps.ptrdbl_ == std::numeric_limits<double>::epsilon();
    //
    base_alloc min = Value( CppAD::numeric_limits<my_ad>::min() );
    ok            &= *min.ptrdbl_ == std::numeric_limits<double>::min();
    //
    base_alloc max = Value( CppAD::numeric_limits<my_ad>::max() );
    ok            &= *max.ptrdbl_ == std::numeric_limits<double>::max();
    //
    base_alloc nan = Value( CppAD::numeric_limits<my_ad>::quiet_NaN() );
    ok            &= *nan.ptrdbl_ != *nan.ptrdbl_;
    //
    int   digits10 = CppAD::numeric_limits<my_ad>::digits10;
    ok            &= digits10 == std::numeric_limits<double>::digits10;
    //
    return ok;
}

bool test_to_string(void)
{   bool ok = true;
    typedef CppAD::AD<base_alloc> my_ad;
    //
    double      dbl_pi = 4.0 * std::atan(1.0);
    my_ad       ad_pi  = my_ad(dbl_pi);
    std::string str_pi = to_string( ad_pi );
    //
    // Check the length of the string "3.1415...". One extra character
    // for machine epsilon precision and another for the decimal point.
    ok &= str_pi.size() == size_t( 2 + std::numeric_limits<double>::digits10 );
    //
    // check value
    double eps   = std::numeric_limits<double>::epsilon();
    double check = std::atof( str_pi.c_str() );
    ok          &= (check / dbl_pi - 1.0) <= eps;
    //
    return ok;
}


} // END empty namespace

bool base_alloc_test(void)
{   bool ok = true;
    //
    ok     &= test_parameter();
    ok     &= test_numeric_limits();
    ok     &= test_to_string();
    //
    return ok;
}
