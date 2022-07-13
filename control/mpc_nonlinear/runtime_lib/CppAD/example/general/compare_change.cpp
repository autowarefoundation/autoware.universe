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
$begin compare_change.cpp$$
$spell
    Cpp
$$

$section CompareChange and Re-Tape: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace { // put this function in the empty namespace
    template <class Type>
    Type Minimum(const Type &x, const Type &y)
    {   // Use a comparison to compute the min(x, y)
        // (note that CondExp would never require retaping).
        if( x < y )
            return x;
        return y;
    }
    struct error_info {
        bool known;
        int  line;
        std::string file;
        std::string exp;
        std::string msg;
    };
    void error_handler(
        bool        known       ,
        int         line        ,
        const char *file        ,
        const char *exp         ,
        const char *msg         )
    {   // error handler must not return, so throw an exception
        error_info info;
        info.known = known;
        info.line  = line;
        info.file  = file;
        info.exp   = exp;
        info.msg   = msg;
        throw info;
    }

}

bool compare_change(void)
{   bool ok = true;
    using CppAD::AD;

    // domain space vector
    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    ax[0] = 3.;
    ax[1] = 4.;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = Minimum(ax[0], ax[1]);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // set count to one (not necessry because is its default value)
    f.compare_change_count(1);

    // evaluate zero mode Forward where comparison has the same result
    // as during taping; i.e., x[0] < x[1].
    CPPAD_TESTVECTOR(double) x(n), y(m);
    x[0] = 2.;
    x[1] = 3.;
    y    = f.Forward(0, x);
    ok  &= (y[0] == x[0]);
    ok  &= (y[0] == Minimum(x[0], x[1]));
    ok  &= (f.compare_change_number() == 0);
    ok  &= (f.compare_change_op_index() == 0);

    // evaluate zero mode Forward where comparison has different result
    // as during taping; i.e., x[0] >= x[1].
    x[0] = 3.;
    x[1] = 2.;
    y    = f.Forward(0, x);
    ok  &= (y[0] == x[0]);
    ok  &= (y[0] != Minimum(x[0], x[1]));
    ok  &= (f.compare_change_number() == 1);
    ok  &= (f.compare_change_op_index() > 0 );
    size_t op_index = f.compare_change_op_index();

    // Local block during which default CppAD error handler is replaced.
    // If you do not replace the default CppAD error handler,
    // and you run in the debugger, you will be able to inspect the
    // call stack and see that 'if( x < y )' is where the comparison is.
    bool missed_error = true;
    {   CppAD::ErrorHandler local_error_handler(error_handler);

        std::string check_msg =
            "Operator index equals abort_op_index in Independent";
        try {
            // determine the operation index where the change occurred
            CppAD::Independent(ax, op_index);
            ay[0] = Minimum(ax[0], ax[1]);
# ifdef NDEBUG
            // CppAD does not spend time checking operator index when
            // NDEBUG is defined
            missed_error = false;
            AD<double>::abort_recording();
# endif
        }
        catch( error_info info )
        {   missed_error = false;
            ok          &= info.known;
            ok          &= info.msg == check_msg;
            // Must abort the recording so we can start a new one
            // (and to avoid a memory leak).
            AD<double>::abort_recording();
        }
    }
# ifdef CPPAD_DEBUG_AND_RELEASE
    if( missed_error )
    {   // This routine is compiled for debugging, but the routine that checks
        // operator indices was compiled for release.
        missed_error = false;
        AD<double>::abort_recording();
    }
# endif
    ok &= ! missed_error;

    // set count to zero to demonstrate case where comparisons are not checked
    f.compare_change_count(0);
    y    = f.Forward(0, x);
    ok  &= (y[0] == x[0]);
    ok  &= (y[0] != Minimum(x[0], x[1]));
    ok  &= (f.compare_change_number()   == 0);
    ok  &= (f.compare_change_op_index() == 0);

    // now demonstrate that compare_change_number works for an optimized
    // tape (note that compare_change_op_index is always zero after optimize)
    f.optimize();
    f.compare_change_count(1);
    y    = f.Forward(0, x);
    ok  &= (y[0] == x[0]);
    ok  &= (y[0] != Minimum(x[0], x[1]));
    ok  &= (f.compare_change_number()   == 1);
    ok  &= (f.compare_change_op_index() == 0);

    // now retape to get the a tape that agrees with the algorithm
    ax[0] = x[0];
    ax[1] = x[1];
    Independent(ax);
    ay[0] = Minimum(ax[0], ax[1]);
    f.Dependent(ax, ay);
    y    = f.Forward(0, x);
    ok  &= (y[0] == x[1]);
    ok  &= (y[0] == Minimum(x[0], x[1]));
    ok  &= (f.compare_change_number()   == 0);
    ok  &= (f.compare_change_op_index() == 0);

    return ok;
}


// END C++
