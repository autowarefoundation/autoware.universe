/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin simple_vector.cpp$$
$spell
    Cpp
$$

$section Simple Vector Template Class: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
--------------------------------------------------------------------------
*/
// BEGIN C++
# include <iostream>                   // std::cout and std::endl

# include <vector>                     // std::vector
# include <valarray>                   // std::valarray
# include <cppad/utility/vector.hpp>       // CppAD::vector
# include <cppad/utility/check_simple_vector.hpp>  // CppAD::CheckSimpleVector
namespace {
    template <class Vector>
    bool Ok(void)
    {   // type corresponding to elements of Vector
        typedef typename Vector::value_type Scalar;

        bool ok = true;             // initialize testing flag

        Vector x;                   // use the default constructor
        ok &= (x.size() == 0);      // test size for an empty vector
        Vector y(2);                // use the sizing constructor
        ok &= (y.size() == 2);      // size for an vector with elements

        // non-const access to the elements of y
        size_t i;
        for(i = 0; i < 2; i++)
            y[i] = Scalar(i);

        const Vector z(y);          // copy constructor
        x.resize(2);                // resize
        x = z;                      // vector assignment

        // use the const access to the elements of x
        // and test the values of elements of x, y, z
        for(i = 0; i < 2; i++)
        {   ok &= (x[i] == Scalar(i));
            ok &= (y[i] == Scalar(i));
            ok &= (z[i] == Scalar(i));
        }
        return ok;
    }
}
bool SimpleVector (void)
{   bool ok = true;

    // use routine above to check these cases
    ok &= Ok< std::vector<double> >();
    ok &= Ok< std::valarray<float> >();
    ok &= Ok< CppAD::vector<int> >();
# ifndef _MSC_VER
    // Avoid Microsoft following compiler warning:  'size_t' :
    // forcing value to bool 'true' or 'false' (performance warning)
    ok &= Ok< std::vector<bool> >();
    ok &= Ok< CppAD::vector<bool> >();
# endif
    // use CheckSimpleVector for more extensive testing
    CppAD::CheckSimpleVector<double, std::vector<double>  >();
    CppAD::CheckSimpleVector<float,  std::valarray<float> >();
    CppAD::CheckSimpleVector<int,    CppAD::vector<int>   >();
    CppAD::CheckSimpleVector<bool,   std::vector<bool>    >();
    CppAD::CheckSimpleVector<bool,   CppAD::vector<bool>  >();

    return ok;
}
// END C++
