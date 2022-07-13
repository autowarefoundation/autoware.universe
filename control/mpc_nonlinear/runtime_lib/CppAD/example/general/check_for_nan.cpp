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
$begin check_for_nan.cpp$$

$section ADFun Checking For Nan: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include <cctype>

namespace {
    void myhandler(
        bool known       ,
        int  line        ,
        const char *file ,
        const char *exp  ,
        const char *msg  )
    {   // error handler must not return, so throw an exception
        std::string message = msg;
        throw message;
    }
}

bool check_for_nan(void)
{   bool ok = true;
    using CppAD::AD;
    using std::string;
    double eps = 10. * std::numeric_limits<double>::epsilon();

    // replace the default CppAD error handler
    CppAD::ErrorHandler info(myhandler);

    CPPAD_TESTVECTOR(AD<double>) ax(2), ay(2);
    ax[0] = 2.0;
    ax[1] = 1.0;
    CppAD::Independent(ax);
    ay[0] = sqrt( ax[0] );
    ay[1] = sqrt( ax[1] );
    CppAD::ADFun<double> f(ax, ay);

    CPPAD_TESTVECTOR(double) x(2), y(2);
    x[0] = 2.0;
    x[1] = -1.0;

    // use try / catch because this causes an exception
    // (assuming that NDEBUG is not defined)
    f.check_for_nan(true);
    try {
        y = f.Forward(0, x);

# ifndef NDEBUG
        // When compiled with NDEBUG defined,
        // CppAD does not spend time checking for nan.
        ok = false;
# endif
    }
    catch(std::string msg)
    {
        // get and check size of the independent variable vector
        string pattern = "vector_size = ";
        size_t start   = msg.find(pattern) + pattern.size();
        string number;
        for(size_t i = start; msg[i] != '\n'; i++)
            number += msg[i];
        size_t vector_size = size_t( std::atoi(number.c_str()) );
        ok &= vector_size == 2;

        // get and check first dependent variable index that is nan
        pattern = "index = ";
        start   = msg.find(pattern) + pattern.size();
        number  = "";
        for(size_t i = start; msg[i] != '\n'; i++)
            number += msg[i];
        size_t index = size_t( std::atoi(number.c_str()) );
        ok &= index == 1;

        // get the name of the file
        pattern = "file_name = ";
        start   = msg.find(pattern) + pattern.size();
        string file_name;
        for(size_t i = start; msg[i] != '\n'; i++)
            file_name += msg[i];

        // get and check independent variable vector that resulted in the nan
        CppAD::vector<double> vec(vector_size);
        CppAD::get_check_for_nan(vec, file_name);
        for(size_t i = 0; i < vector_size; i++)
            ok &= vec[i] == x[i];
    }

    // now do calculation without an exception
    f.check_for_nan(false);
    y = f.Forward(0, x);
    ok &= CppAD::NearEqual(y[0], std::sqrt(x[0]), eps, eps);
    ok &= CppAD::isnan( y[1] );

    return ok;
}

// END C++
