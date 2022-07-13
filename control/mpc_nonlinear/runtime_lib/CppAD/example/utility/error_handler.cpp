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
$begin error_handler.cpp$$
$spell
    CppAD
$$

$section Replacing The CppAD Error Handler: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/utility/error_handler.hpp>
# include <cstring>

namespace {
    void myhandler(
        bool known       ,
        int  line        ,
        const char *file ,
        const char *exp  ,
        const char *msg  )
    {   // error handler must not return, so throw an exception
        throw line;
    }
}


bool ErrorHandler(void)
{   using CppAD::ErrorHandler;

    int lineMinusFive = 0;

    // replace the default CppAD error handler
    ErrorHandler info(myhandler);

    // set ok to false unless catch block is executed
    bool ok = false;

    // use try / catch because handler throws an exception
    try {
        // set the static variable Line to next source code line
        lineMinusFive = __LINE__;

        // can call myhandler anywhere that ErrorHandler is defined
        ErrorHandler::Call(
            true     , // reason for the error is known
            __LINE__ , // current source code line number
            __FILE__ , // current source code file name
            "1 > 0"  , // an intentional error condition
            "Testing ErrorHandler"     // reason for error
        );
    }
    catch ( int line )
    {   // check value of the line number that was passed to handler
        ok = (line == lineMinusFive + 5);
    }

    // info drops out of scope and the default CppAD error handler
    // is restored when this routine returns.
    return ok;
}

// END C++
