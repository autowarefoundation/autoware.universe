/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin fun_property.cpp$$
$spell
    op
    arg
    abs
    var
    VecAD
$$

$section ADFun Function Properties: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

// Note that CPPAD_VEC_ENUM_TYPE is not part of CppAD API and may change
# define CPPAD_VEC_ENUM_TYPE unsigned char

bool fun_property(void)
{   bool ok = true;
    using CppAD::AD;

    // Use nvar to track the number of variables in the operation sequence.
    // Start with one for the phantom variable at tape address zero.
    size_t nvar = 1;

    // Use npar to track the number of parameters in the operation sequence.
    // Start with one for the phantom parameter at index zero.
    size_t npar = 1;

    // Use ndyn to track the number of dynamic parameters.
    size_t ndyn = 0;

    // Use ndyn to track number of arguments to dynamic parameter operators.
    size_t ndyn_arg = 0;

    // Start with one for operator corresponding to phantom variable
    size_t nop  = 1;

    // Start with one for operator corresponding to phantom argument
    size_t narg = 1;

    // Use ntext to track the number of characters used to label
    // output generated using PrintFor commands.
    size_t ntext = 0;

    // Use nvecad to track the number of VecAD vectors, plus the number
    // of VecAD vector elements, in the operation sequence.
    size_t nvecad = 0;

    // a VecAD vector
    CppAD::VecAD<double> v(2);
    v[0]     = 0; // requires the parameter 0, when becomes a variable
    v[1]     = 1; // requires the parameter 1, when becomes a variable

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]     = 0.;
    x[1]     = 1.;

    // dynamic parameter vector
    CPPAD_TESTVECTOR(AD<double>) dynamic(1);
    dynamic[0] = 1.;


    // declare independent variables and start tape recording
    size_t abort_op_index = 0;
    bool   record_compare = true;
    CppAD::Independent(x, abort_op_index, record_compare, dynamic);
    nvar    += n;
    nop     += n;
    ndyn    += dynamic.size();
    npar    += ndyn;

    // a computation that adds to the operation sequence
    AD<double> I = 0;
    v[I]         = x[0];
    nvecad      +=   3;  // one for vector, two for its elements
    npar        +=   2;  // need parameters 0 and 1 for initial v
    nop         +=   1;  // operator for storing in a VecAD object
    narg        +=   3;  // the three arguments are v, I, and x[0]

    // some operations that do not add to the operation sequence
    AD<double> u = x[0];  // use same variable as x[0]
    AD<double> w = x[1];  // use same variable as x[1]

    // a computation that adds to the operation sequence
    w      = w * (u + w);
    nop   += 2;   // requires two new operators, an add and a multiply
    nvar  += 2;   // each operator results in its own variable
    narg  += 4;   // each operator has two arguments

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) y(m);

    // operations that do not add to the operation sequence
    y[0]   = 1.;  // re-use the parameter 1
    y[1]   = u;   // use same variable as u

    // a computation that adds to the operation sequence
    y[2]   = w + 2.;
    nop   += 1;   // requires a new add operator
    npar  += 1;   // new parameter 2 is new, so it must be included
    nvar  += 1;   // variable corresponding to the result
    narg  += 2;   // operator has two arguments

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);
    nop   += 1;   // special operator for y[0] because it is a parameter
    nvar  += 1;   // special variable for y[0] because it is a parameter
    narg  += 1;   // identifies which parameter corresponds to y[0]
    nop   += 1;   // special operator at the end of operation sequence

    // check the sequence property functions
    ok &= f.Domain()        == n;
    ok &= f.Range()         == m;
    ok &= f.Parameter(0)    == true;
    ok &= f.Parameter(1)    == false;
    ok &= f.Parameter(2)    == false;
    ok &= f.size_var()      == nvar;
    ok &= f.size_op()       == nop;
    ok &= f.size_op_arg()   == narg;
    ok &= f.size_par()      == npar;
    ok &= f.size_text()     == ntext;
    ok &= f.size_VecAD()    == nvecad;
    ok &= f.size_dyn_ind()  == ndyn;
    ok &= f.size_dyn_par()  == ndyn;
    ok &= f.size_dyn_arg()  == ndyn_arg;

    //
    size_t sum = 0;
    sum += nop        * sizeof(CPPAD_VEC_ENUM_TYPE);
    sum += narg       * sizeof(CPPAD_TAPE_ADDR_TYPE);
    sum += npar       * sizeof(double);
    sum += npar       * sizeof(bool);
    sum += ndyn       * sizeof(CPPAD_VEC_ENUM_TYPE);
    sum += ndyn       * sizeof(CPPAD_TAPE_ADDR_TYPE);
    sum += ndyn_arg   * sizeof(CPPAD_TAPE_ADDR_TYPE);
    sum += ntext      * sizeof(char);
    sum += nvecad     * sizeof(CPPAD_TAPE_ADDR_TYPE);
    ok &= f.size_op_seq() == sum;

    return ok;
}

// END C++
