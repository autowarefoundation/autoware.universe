/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// Complex examples should supppress conversion warnings
# include <cppad/wno_conversion.hpp>
//
# include <cppad/cppad.hpp>

namespace {
    template <class Float>
    bool test(void)
    {   bool  ok  = true;
        Float eps = std::numeric_limits<Float>::epsilon();
        Float pi  = Float( 4.0 * std::atan(1.0) );
        Float e   = Float( std::exp(1.0) );
        typedef std::complex<Float> Base;
        Base  c   = Base(pi, e);
        std::string s = CppAD::to_string( CppAD::AD<Base>(c) );
        //
        // get strings corresponding to real and imaginary parts
        std::string real = "";
        std::string imag = "";
        size_t index = 1; // skip ( at front
        while( s[index] != ',' )
            real += s[index++];
        index++;
        while( s[index] != ')' )
            imag += s[index++];
        //
        Float check   = Float( std::atof( real.c_str() ) );
        ok           &= std::fabs( check / pi - 1.0 ) <= 2.0 * eps;
        //
        check         = Float( std::atof( imag.c_str() ) );
        ok           &= std::fabs( check / e - 1.0 ) <= 2.0 * eps;
        //
        return ok;
    }
}

bool to_string(void)
{   bool ok = true;

    ok &= test<float>();
    ok &= test<double>();

    return ok;
}

// END C++
