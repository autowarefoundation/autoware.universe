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
$begin time_test.cpp$$
$spell
    cppad.hpp
    Microsoft
    namespace
    std
    const
    cout
    ctime
    ifdef
    const
    endif
    cpp
$$


$section time_test: Example and test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/utility/time_test.hpp>
# include <cppad/utility/vector.hpp>

namespace { // empty namespace
    using CppAD::vector;

    // used to check size
    size_t size_;

    // used to check repeat
    size_t repeat_;

    vector<double> a, b, c;
    void test(size_t size, size_t repeat)
    {   // used for check
        size_   = size;
        repeat_ = repeat;

        // setup
        a.resize(size);
        b.resize(size);
        c.resize(size);
        size_t i  = size;;
        while(i)
        {   --i;
            a[i] = float(i);
            b[i] = float(2 * i);
            c[i] = 0.0;
        }
        // operations we are timing
        while(repeat--)
        {   i = size;;
            while(i)
            {   --i;
                c[i] += std::sqrt(a[i] * a[i] + b[i] * b[i]);
            }
        }
    }

}
bool time_test(void)
{   bool ok = true;
    using CppAD::time_test;

    // minimum amount of time to run test
    double time_min = 0.5;

    // size of first test case
    size_t test_size = 20;

    // run the first test case
    size_t repeat_first;
    double time_first = time_test(test, time_min, test_size, repeat_first);
    ok &= size_   == test_size;
    ok &= repeat_ == repeat_first;
    ok &= time_min <= double(repeat_first)  * time_first;

    // size of second test case is twice as large
    test_size = 2 * test_size;

    // run the second test case
    size_t repeat_second;
    double time_second = time_test(test, time_min, test_size, repeat_second);
    ok &= size_   == test_size;
    ok &= repeat_ == repeat_second;
    ok &= time_min <= double(repeat_second) * time_second;

    // test above should hold without exception, one below might not
    assert( ok );

    // for this case, time should be linear w.r.t size
    double rel_diff = 1. - 2. * time_first / time_second;
    ok             &= (std::fabs(rel_diff) <= .1);
    if( ! ok )
        std::cout << std::endl << "rel_diff = " << rel_diff  << std::endl;

    a.clear();
    b.clear();
    c.clear();
    return ok;
}
// END C++
