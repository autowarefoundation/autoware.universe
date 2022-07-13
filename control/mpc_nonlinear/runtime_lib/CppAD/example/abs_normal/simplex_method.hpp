# ifndef CPPAD_EXAMPLE_ABS_NORMAL_SIMPLEX_METHOD_HPP
# define CPPAD_EXAMPLE_ABS_NORMAL_SIMPLEX_METHOD_HPP
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
$begin simplex_method$$
$spell
    hpp
    maxitr
    xout
$$

$section abs_normal: Solve a Linear Program Using Simplex Method$$

$head Syntax$$
$icode%ok% = simplex_method(%level%, %b%, %A%, %c%, %maxitr%, %xout%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%
1%$$

$head Source$$
This following is a link to the source code for this example:
$cref/simplex_method.hpp/simplex_method.hpp/$$.

$head Problem$$
We are given
$latex A \in \B{R}^{m \times n}$$,
$latex b \in \B{R}^m$$,
$latex c \in \B{R}^n$$.
This routine solves the problem
$latex \[
\begin{array}{rl}
\R{minimize} &
g^T x \; \R{w.r.t} \; x \in \B{R}_+^n
\\
\R{subject \; to} & A x + b \leq 0
\end{array}
\] $$

$head Vector$$
The type $icode Vector$$ is a
simple vector with elements of type $code double$$.

$head level$$
This value is less than or equal two.
If $icode%level% == 0%$$,
no tracing is printed.
If $icode%level% >= 1%$$,
a trace $latex x$$ and the corresponding objective $latex z$$
is printed at each iteration.
If $icode%level% == 2%$$,
a trace of the simplex Tableau is printed at each iteration.

$head A$$
This is a $cref/row-major/glossary/Row-major Representation/$$ representation
of the matrix $latex A$$ in the problem.

$head b$$
This is the vector $latex b$$ in the problem.

$head c$$
This is the vector $latex c$$ in the problem.

$head maxitr$$
This is the maximum number of simplex iterations to try before giving up
on convergence.

$head xout$$
This argument has size is $icode n$$ and
the input value of its elements does no matter.
Upon return it is the primal variables corresponding to the problem solution.

$head ok$$
If the return value $icode ok$$ is true, a solution has been found.

$children%example/abs_normal/simplex_method.cpp
    %example/abs_normal/simplex_method.omh
%$$
$head Example$$
The file $cref simplex_method.cpp$$ contains an example and test of
$code simplex_method$$.

$end
-----------------------------------------------------------------------------
*/
# include <cmath>
# include <cppad/utility/error_handler.hpp>
# include "abs_print_mat.hpp"

// BEGIN C++
namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// BEGIN PROTOTYPE
template <class Vector>
bool simplex_method(
    size_t        level   ,
    const Vector& A       ,
    const Vector& b       ,
    const Vector& c       ,
    size_t        maxitr  ,
    Vector&       xout    )
// END PROTOTYPE
{   // number of equations
    size_t ne  = b.size();
    // number of x variables
    size_t nx = c.size();
    CPPAD_ASSERT_UNKNOWN( size_t(A.size()) == ne * nx );
    CPPAD_ASSERT_UNKNOWN( level <= 2 );
    //
    if( level > 0 )
    {   std::cout << "start simplex_method\n";
        CppAD::abs_print_mat("A", ne, nx, A);
        CppAD::abs_print_mat("b", ne,  1, b);
        CppAD::abs_print_mat("c", nx, 1, c);
    }
    //
    // variables (columns) in the Tableau:
    // x: the original primary variables with size n
    // s: slack variables, one for each equation
    // a: auxillary variables, one for each negative right hand size
    // r: right hand size for equations
    //
    // Determine number of auxillary variables
    size_t na = 0;
    for(size_t i = 0; i < ne; i++)
    {   if( b[i] > 0.0 )
            ++na;
    }
    // size of columns in the Tableau
    size_t nc = nx + ne + na + 1;

    // number of rows in Tableau, the equations plust two objectives
    size_t nr = ne + 2;

    // Initilize Tableau as zero
    Vector T(nr * nc);
    for(size_t i = 0; i < nr * nc; i++)
        T[i] = 0.0;

    // initialize basic variable flag as false
    CppAD::vector<size_t> basic(nc);
    for(size_t j = 0; j < nc; j++)
        basic[j] = false;

    // For i = 0 , ... , m-1, place the Equations
    // sum_j A_{i,j} * x_j + b_i <= 0 in Tableau
    na = 0; // use as index of next auxillary variable
    for(size_t i = 0; i < ne; i++)
    {   if( b[i] > 0.0)
        {   // convert to - sum_j A_{i,j} x_j - b_i >= 0
            for(size_t j = 0; j < nx; j++)
                T[i * nc + j] = - A[i * nx + j];
            // slack variable has negative coefficient
            T[i * nc + (nx + i)] = -1.0;
            // auxillary variable is basic for this constraint
            T[i * nc + (nx + ne + na)] = 1.0;
            basic[nx + ne + na]        = true;
            // right hand side
            T[i * nc + (nc - 1)] = b[i];
            //
            ++na;
        }
        else
        {   // sum_j A_{i,j} x_j + b_i <= 0
            for(size_t j = 0; j < nx; j++)
                T[i * nc + j] = A[i * nx + j];
            //  slack variable is also basic
            T[ i * nc + (nx + i) ]  = 1.0;
            basic[nx + i]           = true;
            // right hand side for equations
            T[ i * nc + (nc - 1) ] = - b[i];
        }
    }
    // na is back to its original value
    CPPAD_ASSERT_UNKNOWN( nc == nx + ne + na + 1 );
    //
    // place the equation objective equation in Tablueau
    // row ne corresponds to the equation z - sum_j c_j x_j = 0
    // column index for z is nx + ne + na
    for(size_t j = 0; j < nx; j++)
        T[ne * nc + j] = - c[j];
    //
    // row ne+1 corresponds to the equation w - a_0 - ... - a_{na-1} = 0
    // column index for w is nx + ne + na +1
    for(size_t j = 0; j < na; j++)
        T[(ne + 1) * nc + (nx + ne + j)] = -1.0;
    //
    // fix auxillary objective so coefficients in w
    // for auxillary variables are zero
    for(size_t k = 0; k < na; k++)
    {   size_t ja  = nx + ne + k;
        size_t ia  = ne;
        for(size_t i = 0; i < ne; i++)
        {   if( T[i * nc + ja] != 0.0 )
            {   CPPAD_ASSERT_UNKNOWN( T[i * nc + ja] == 1.0 );
                CPPAD_ASSERT_UNKNOWN( T[(ne + 1) * nc + ja] == -1.0 )
                CPPAD_ASSERT_UNKNOWN( ia == ne );
                ia = i;
            }
        }
        CPPAD_ASSERT_UNKNOWN( ia < ne );
        for(size_t j = 0; j < nc; j++)
            T[(ne + 1) * nc + j] += T[ia * nc + j];
        // The result in column ja is zero, avoid roundoff
        T[(ne + 1) * nc + ja] = 0.0;
    }
    //
    // index of current objective
    size_t iobj = ne;  // original objective z
    if( na > 0 )
        iobj = ne + 1; // auxillary objective w
    //
    // simplex interations
    for(size_t itr = 0; itr < maxitr; itr++)
    {   // current value for xout
        for(size_t j = 0; j < nx; j++)
        {   xout[j] = 0.0;
            if( basic[j] )
            {   // determine which row of column j is non-zero
                xout[j] = std::numeric_limits<double>::quiet_NaN();
                for(size_t i = 0; i < ne; i++)
                {   double T_ij = T[i * nc + j];
                    CPPAD_ASSERT_UNKNOWN( T_ij == 0.0 || T_ij == 1.0 );
                    if( T_ij == 1.0 )
                    {   // corresponding value in right hand side
                        xout[j] = T[ i * nc + (nc-1) ];
                    }
                }
            }
        }
        if( level > 1 )
            CppAD::abs_print_mat("T", nr, nc, T);
        if( level > 0 )
        {   CppAD::abs_print_mat("x", nx, 1, xout);
            std::cout << "itr = " << itr;
            if( iobj > ne )
                std::cout << ", auxillary objective w = ";
            else
                std::cout << ", objective z = ";
            std::cout << T[iobj * nc + (nc - 1)] << "\n";
        }
        //
        // number of variables depends on objective
        size_t nv = nx + ne;   // (x, s)
        if( iobj == ne + 1 )
        {   // check if we have solved the auxillary problem
            bool done = true;
            for(size_t k = 0; k < na; k++)
                if( basic[nx + ne + k] )
                    done = false;
            if( done )
            {   // switch to optimizing the original objective
                iobj = ne;
            }
            else
                nv = nx + ne + na; // (x, s, a)
        }
        //
        // determine variable with maximuim coefficient in objective row
        double cmax = 0.0;
        size_t jmax = nv;
        for(size_t j = 0; j < nv; j++)
        {   if( T[iobj * nc + j] > cmax )
            {   CPPAD_ASSERT_UNKNOWN( ! basic[j] );
                cmax = T[ iobj * nc + j];
                jmax = j;
            }
        }
        // check for solution
        if( jmax == nv )
        {   if( iobj == ne )
            {   if( level > 0 )
                    std::cout << "end simplex_method\n";
                return true;
            }
            if( level > 0 )
                std::cout << "end_simples_method: no feasible solution\n";
            return false;
        }
        //
        // We will increase the j-th variable.
        // Determine which row will be the pivot row.
        double rmin = std::numeric_limits<double>::infinity();
        size_t imin = ne;
        for(size_t i = 0; i < ne; i++)
        {   if( T[i * nc + jmax] > 0.0 )
            {   double r = T[i * nc + (nc-1) ] / T[i * nc + jmax];
                if( r < rmin )
                {   rmin = r;
                    imin = i;
                }
            }
        }
        if( imin == ne )
        {   // not auxillary objective
            CPPAD_ASSERT_UNKNOWN( iobj == ne );
            if( level > 0 ) std::cout
                << "end simplex_method: objective is unbounded below\n";
            return false;
        }
        double pivot = T[imin * nc + jmax];
        //
        // Which variable is changing from basic to non-basic.
        // Initilaize as not yet determined.
        size_t basic2not = nc;
        //
        // Divide row imin by pivot element
        for(size_t j = 0; j < nc; j++)
        {   if( basic[j] && T[imin * nc + j] == 1.0 )
            {   CPPAD_ASSERT_UNKNOWN( basic2not == nc );
                basic2not = j;
            }
            T[imin * nc + j] /= pivot;
        }
        // The result in column jmax is one, avoid roundoff
        T[imin * nc + jmax ] = 1.0;
        //
        // Check that we found the variable going from basic to non-basic
        CPPAD_ASSERT_UNKNOWN( basic2not < nv && basic2not != jmax );
        //
        // convert variable for column jmax to basic
        // and for column basic2not to non-basic
        for(size_t i = 0; i < nr; i++) if( i != imin )
        {   double r = T[i * nc + jmax ] / T[imin * nc + jmax];
            // row_i = row_i - r * row_imin
            for(size_t j = 0; j < nc; j++)
                T[i * nc + j] -= r * T[imin * nc + j];
            // The result in column jmax is zero, avoid roundoff
            T[i * nc + jmax] = 0.0;
        }
        // update flag for basic variables
        basic[ basic2not ] = false;
        basic[ jmax ]      = true;
    }
    if( level > 0 ) std::cout
        << "end simplex_method: maximum # iterations without solution\n";
    return false;
}
} // END_CPPAD_NAMESPACE
// END C++

# endif
