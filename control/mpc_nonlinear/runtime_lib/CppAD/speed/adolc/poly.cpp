/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin adolc_poly.cpp$$
$spell
    alloc
    onetape
    coef
    cppad
    hos
    Taylor
    std
    ddp
    cppad
    adouble
    std
    vector Vector
    typedef
    adolc
    Lu
    CppAD
    det
    hpp
    const
    bool
$$

$section Adolc Speed: Second Derivative of a Polynomial$$


$head Specifications$$
See $cref link_poly$$.

$head Implementation$$

$srccode%cpp% */
// suppress conversion warnings before other includes
# include <cppad/wno_conversion.hpp>
//
# include <vector>
# include <adolc/adolc.h>

# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/poly.hpp>
# include <cppad/utility/vector.hpp>
# include <cppad/utility/thread_alloc.hpp>
# include "adolc_alloc_mat.hpp"

// list of possible options
# include <map>
extern std::map<std::string, bool> global_option;

bool link_poly(
    size_t                     size     ,
    size_t                     repeat   ,
    CppAD::vector<double>     &a        ,  // coefficients of polynomial
    CppAD::vector<double>     &z        ,  // polynomial argument value
    CppAD::vector<double>     &ddp      )  // second derivative w.r.t z
{
    if( global_option["atomic"] )
        return false;
    if( global_option["memory"] || global_option["optimize"] )
        return false;
    // -----------------------------------------------------
    // setup
    size_t i;
    int tag  = 0;  // tape identifier
    int keep = 0;  // do not keep forward mode results in buffer
    int m    = 1;  // number of dependent variables
    int n    = 1;  // number of independent variables
    int d    = 2;  // highest derivative degree
    double f;      // function value

    // set up for thread_alloc memory allocator (fast and checks for leaks)
    using CppAD::thread_alloc; // the allocator
    size_t capacity;           // capacity of an allocation

    // choose a vector of polynomial coefficients
    CppAD::uniform_01(size, a);

    // AD copy of the polynomial coefficients
    std::vector<adouble> A(size);
    for(i = 0; i < size; i++)
        A[i] = a[i];

    // domain and range space AD values
    adouble Z, P;

    // allocate arguments to hos_forward
    double* x0 = thread_alloc::create_array<double>(size_t(n), capacity);
    double* y0 = thread_alloc::create_array<double>(size_t(m), capacity);
    double** x = adolc_alloc_mat(size_t(n), size_t(d));
    double** y = adolc_alloc_mat(size_t(m), size_t(d));

    // Taylor coefficient for argument
    x[0][0] = 1.;  // first order
    x[0][1] = 0.;  // second order

    // ----------------------------------------------------------------------
    if( ! global_option["onetape"] ) while(repeat--)
    {   // choose an argument value
        CppAD::uniform_01(1, z);

        // declare independent variables
        trace_on(tag, keep);
        Z <<= z[0];

        // AD computation of the function value
        P = CppAD::Poly(0, A, Z);

        // create function object f : Z -> P
        P >>= f;
        trace_off();

        // set the argument value
        x0[0] = z[0];

        // evaluate the polynomial at the new argument value
        hos_forward(tag, m, n, d, keep, x0, x, y0, y);

        // second derivative is twice second order Taylor coef
        ddp[0] = 2. * y[0][1];
    }
    else
    {
        // choose an argument value
        CppAD::uniform_01(1, z);

        // declare independent variables
        trace_on(tag, keep);
        Z <<= z[0];

        // AD computation of the function value
        P = CppAD::Poly(0, A, Z);

        // create function object f : Z -> P
        P >>= f;
        trace_off();

        while(repeat--)
        {   // get the next argument value
            CppAD::uniform_01(1, z);
            x0[0] = z[0];

            // evaluate the polynomial at the new argument value
            hos_forward(tag, m, n, d, keep, x0, x, y0, y);

            // second derivative is twice second order Taylor coef
            ddp[0] = 2. * y[0][1];
        }
    }
    // ------------------------------------------------------
    // tear down
    adolc_free_mat(x);
    adolc_free_mat(y);
    thread_alloc::delete_array(x0);
    thread_alloc::delete_array(y0);

    return true;
}
/* %$$
$end
*/
