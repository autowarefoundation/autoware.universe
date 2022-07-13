/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
Test of Eigen Interface to CppAD Scalar Types
$end
*/
# include <cppad/example/cppad_eigen.hpp>

bool cppad_eigen(void)
{   bool ok = true;
    using CppAD::AD;
    using Eigen::Dynamic;
    using Eigen::Matrix;

    typedef Eigen::NumTraits<AD<double> >         traits;

    ok &= traits::IsComplex              == 0;
    ok &= traits::IsInteger              == 0;
    ok &= traits::IsSigned               == 1;
    ok &= traits::RequireInitialization  == 1;
    ok &= traits::ReadCost               == 1;
    ok &= traits::AddCost                == 2;
    ok &= traits::MulCost                == 2;

    ok &= traits::epsilon() ==
        std::numeric_limits<double>::epsilon();
    ok &= traits::dummy_precision() ==
        100.* std::numeric_limits<double>::epsilon();
    ok &= traits::highest() ==
        std::numeric_limits<double>::max();
    ok &= traits::lowest() ==
        std::numeric_limits<double>::min();
    ok &= std::isnan(traits::quiet_NaN());
    ok &= std::isinf(traits::infinity());

    AD<double> x = 2.0;
    ok  &= conj(x)  == x;
    ok  &= real(x)  == x;
    ok  &= imag(x)  == 0.0;
    ok  &= abs2(x)  == 4.0;

    ok  &= (!std::isinf(x));
    ok  &= (!std::isnan(x));

    x = traits::quiet_NaN();
    ok  &= std::isnan(x);

    x = traits::infinity();
    ok  &= std::isinf(x);

    // Outputing a matrix used to fail before partial specialization of
    // struct significant_decimals_default_impl in cppad_eigen.hpp.
    Matrix< AD<double>, 1, 1> X;
    X(0, 0) = AD<double>(1);
    std::stringstream stream_out;
    stream_out << X;
    ok &= "1" == stream_out.str();

    // multiplying three matrices together used to cause warning
    // before making ctor from arbitrary type to AD<Base> explicit.
    typedef CppAD::AD<double> AScalar;
    Matrix<AScalar, Dynamic, Dynamic> A(1,1), B(1,1), C(1,1), D(1,1);
    A(0,0) = 1.0;
    B(0,0) = 2.0;
    C(0,0) = 3.0;
    D      = A * B * C;
    ok    &= D(0,0) == 6.0 ;

    return ok;
}
