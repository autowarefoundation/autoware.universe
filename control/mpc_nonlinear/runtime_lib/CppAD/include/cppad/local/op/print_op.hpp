# ifndef CPPAD_LOCAL_OP_PRINT_OP_HPP
# define CPPAD_LOCAL_OP_PRINT_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
Print operation for parameters; i.e., op = PriOp.

The C++ source code corresponding to this operation is
\verbatim
    f.Forward(0, x)
    PrintFor(before, var)
    PrintFor(pos, before, var, after)
\endverbatim
The PrintFor call puts the print operation on the tape
and the print occurs during the zero order forward mode computation.

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param s_out
the results are printed on this output stream.

\param arg
 arg[0] & 1
\n
If this is zero, pos is a parameter. Otherwise it is a variable.
\n
 arg[0] & 2
\n
If this is zero, var is a parameter. Otherwise it is a variable.
\n
\n
 arg[1]
\n
If pos is a parameter, <code>parameter[arg[1]]</code> is its value.
Othwise <code>taylor[ size_t(arg[1]) * cap_order + 0 ]</code> is the zero
order Taylor coefficient for pos.
\n
\n
 arg[2]
\n
index of the text to be printed before var
if pos is not a positive value.
\n
\n
 arg[3]
\n
If var is a parameter, <code>parameter[arg[3]]</code> is its value.
Othwise <code>taylor[ size_t(arg[3]) * cap_order + 0 ]</code> is the zero
order Taylor coefficient for var.
\n
\n
 arg[4]
\n
index of the text to be printed after var
if pos is not a positive value.

\param num_text
is the total number of text characters on the tape
(only used for error checking).

\param text
\b Input: <code>text[arg[1]]</code> is the first character of the text
that will be printed. All the characters from there to (but not including)
the first '\\0' are printed.

\param num_par
is the total number of values in the parameter vector

\param parameter
Contains the value of parameters.

\param cap_order
number of colums in the matrix containing all the Taylor coefficients.

\param taylor
Contains the value of variables.

\par Checked Assertions:
\li NumArg(PriOp)  == 5
\li NumRes(PriOp)  == 0
\li text          !=  nullptr
\li arg[1]         <  num_text
\li if pos is a parameter, arg[1] < num_par
\li if var is a parameter, arg[3] < num_par
*/
template <class Base>
void forward_pri_0(
    std::ostream& s_out       ,
    const addr_t* arg         ,
    size_t        num_text    ,
    const char*   text        ,
    size_t        num_par     ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    const Base*   taylor      )
{   Base pos, var;
    const char* before;
    const char* after;
    CPPAD_ASSERT_NARG_NRES(PriOp, 5, 0);

    // pos
    if( arg[0] & 1 )
    {   pos = taylor[ size_t(arg[1]) * cap_order + 0 ];
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[1]) < num_par );
        pos = parameter[ arg[1] ];
    }

    // before
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < num_text );
    before = text + arg[2];

    // var
    if( arg[0] & 2 )
    {   var = taylor[ size_t(arg[3]) * cap_order + 0 ];
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( size_t(arg[3]) < num_par );
        var = parameter[ arg[3] ];
    }

    // after
    CPPAD_ASSERT_UNKNOWN( size_t(arg[4]) < num_text );
    after = text + arg[4];

    if( ! GreaterThanZero( pos ) )
        s_out << before << var << after;
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
