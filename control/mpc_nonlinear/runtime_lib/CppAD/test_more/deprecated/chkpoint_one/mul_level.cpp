/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin chkpoint_one_mul_level.cpp$$
$spell
    checkpointing
$$

$section Atomic Operations and Multiple-Levels of AD: Example and Test$$

$head Discussion$$
One can use $cref/checkpoint/chkpoint_one/$$ or $cref atomic_two$$ to code
an $codei%AD<%Base%>%$$ operation as atomic.
This means that derivative computations that use the type $icode Base$$
will call the corresponding $code atomic_base$$ member functions.
On the other hand, if $icode Base$$ is $codei%AD<%Other%>%$$ the
operations recorded at the $icode Base$$ level will not be atomic.
This is demonstrated in this example.

$comment %example/chkpoint_one/mul_level.cpp%0%// BEGIN C++%// END C++%1%$$


$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    typedef AD<double>                      a1double;
    typedef AD<a1double>                    a2double;
    typedef CPPAD_TESTVECTOR(a1double)      a1vector;
    typedef CPPAD_TESTVECTOR(a2double)      a2vector;

    void f_algo(const a2vector& x, a2vector& y)
    {   size_t n = x.size();
        y[0] = 0.0;
        for(size_t j = 1; j < n; j++)
            y[0] += x[j-1] * x[j];
        return;
    }
}
//
bool mul_level(void)
{   bool ok = true;
    using CppAD::checkpoint;
    using CppAD::ADFun;
    using CppAD::Independent;

    // domain dimension for this problem
    size_t n = 10;
    size_t m = 1;

    // checkpoint version of the function F(x)
    a2vector a2x(n), a2y(m);
    for(size_t j = 0; j < n; j++)
        a2x[j] = a2double(j + 1);
    //
    // could also use bool_sparsity_enum or set_sparsity_enum
    checkpoint<a1double> atom_f("atom_f", f_algo, a2x, a2y);
    //
    // Record a version of y = f(x) without checkpointing
    Independent(a2x);
    f_algo(a2x, a2y);
    ADFun<a1double> check_not(a2x, a2y);
    //
    // number of variables in a tape of f_algo that does not use checkpointing
    size_t size_not = check_not.size_var();
    //
    // Record a version of y = f(x) with checkpointing
    Independent(a2x);
    atom_f(a2x, a2y);
    ADFun<a1double> check_yes(a2x, a2y);
    //
    // f_algo is represented by one atomic operation in this tape
    ok &= check_yes.size_var() < size_not;
    //
    // now record operations at a1double level
    a1vector a1x(n), a1y(m);
    for(size_t j = 0; j < n; j++)
        a1x[j] = a1double(j + 1);
    //
    // without checkpointing
    Independent(a1x);
    a1y = check_not.Forward(0, a1x);
    ADFun<double> with_not(a1x, a1y);
    //
    // should have the same size
    ok &= with_not.size_var() == size_not;
    //
    // with checkpointing
    Independent(a1x);
    a1y = check_yes.Forward(0, a1x);
    ADFun<double> with_yes(a1x, a1y);
    //
    // f_algo is nolonger represented by one atomic operation in this tape
    ok &= with_yes.size_var() == size_not;
    //
    return ok;
}
// END C++
