# ifndef CPPAD_EXAMPLE_ABS_NORMAL_QP_BOX_HPP
# define CPPAD_EXAMPLE_ABS_NORMAL_QP_BOX_HPP
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
$begin qp_box$$
$spell
    hpp
    const
    col
    xout
    yout
    sout
    cols
    prog
    maxitr
    xin
    qp
$$
$section abs_normal: Solve a Quadratic Program With Box Constraints$$

$head Syntax$$
$icode%ok% = qp_box(
    %level%, %a%, %b%, %c%, %C%, %g%, %G%, %epsilon%, %maxitr%, %xin%, %xout%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%
1%$$

$head Source$$
This following is a link to the source code for this example:
$cref/qp_box.hpp/qp_box.hpp/$$.

$head Purpose$$
This routine could be used to create a version of $cref abs_min_linear$$
that solved quadratic programs (instead of linear programs).

$head Problem$$
We are given
$latex a \in \B{R}^n$$,
$latex b \in \B{R}^n$$,
$latex c \in \B{R}^m$$,
$latex C \in \B{R}^{m \times n}$$,
$latex g \in \B{R}^n$$,
$latex G \in \B{R}^{n \times n}$$,
where $latex G$$ is positive semi-definite.
This routine solves the problem
$latex \[
\begin{array}{rl}
\R{minimize} &
\frac{1}{2} x^T G x + g^T x \; \R{w.r.t} \; x \in \B{R}^n
\\
\R{subject \; to} & C x + c \leq 0 \; \R{and} \; a \leq x \leq b
\end{array}
\] $$
The matrix $latex G + C^T C$$ must be positive definite on components
of the vector $latex x$$ where the lower limit minus infinity
and the upper limit is plus infinity; see $icode a$$ and $icode b$$ below.

$head Vector$$
The type $icode Vector$$ is a
simple vector with elements of type $code double$$.

$head level$$
This value is less that or equal two.
If $icode%level% == 0%$$,
no tracing is printed.
If $icode%level% >= 1%$$,
a trace of the $code qp_box$$ operations is printed.
If $icode%level% == 2%$$,
a trace of the $cref qp_interior$$ sub-problem is printed.

$head a$$
This is the vector of lower limits for $latex x$$ in the problem.
If $icode%a%[%j%]%$$ is minus infinity, there is no lower limit
for $latex x_j$$.

$head b$$
This is the vector of upper limits for $latex x$$ in the problem.
If $icode%a%[%j%]%$$ is plus infinity, there is no upper limit
for $latex x_j$$.

$head c$$
This is the value of the inequality constraint function at $latex x = 0$$.

$head C$$
This is a $cref/row-major/glossary/Row-major Representation/$$ representation
of thee the inequality constraint matrix $latex C$$.

$head g$$
This is the gradient of the objective function.

$head G$$
This is a row-major representation of the Hessian of the objective function.
For $latex j = 0 , \ldots , n-1$$,
$latex - \infty < a_j$$ or
$latex b_j < + \infty$$ or
$latex G_{j,j} > 0.0$$.

$head epsilon$$
This argument is the convergence criteria;
see $cref/KKT conditions/qp_box/KKT Conditions/$$ below.
It must be greater than zero.

$head maxitr$$
This is the maximum number of
$cref qp_interior$$ iterations to try before giving up
on convergence.

$head xin$$
This argument has size $icode n$$ and is the initial point for the algorithm.
It must strictly satisfy the constraints; i.e.,
$codei%
    %a% < %xin%,  %xin% < %b%,  %C% * %xin% - %c% < 0
%$$

$head xout$$
This argument has size is $icode n$$ and
the input value of its elements does no matter.
Upon return it is the primal variables
$latex x$$ corresponding to the problem solution.

$head ok$$
If the return value $icode ok$$ is true, convergence is obtained; i.e.,
$latex \[
    | F ( x , y_a, s_a, y_b, s_b, y_c, s_c ) |_\infty < \varepsilon
\] $$
where $latex |v|_\infty$$ is the infinity norm of the vector $latex v$$,
$latex \varepsilon$$ is $icode epsilon$$,
$latex x$$ is equal to $icode xout$$,
$latex y_a, s_a \in \B{R}_+^n$$,
$latex y_b, s_b \in \B{R}_+^n$$ and
$latex y_c, s_c \in \B{R}_+^m$$.

$head KKT Conditions$$
Give a vector $latex v \in \B{R}^m$$ we define
$latex D(v) \in \B{R}^{m \times m}$$ as the corresponding diagonal matrix.
We also define $latex 1_m \in \B{R}^m$$ as the vector of ones.
We define
$latex \[
F ( x , y_a, s_a, y_b, s_b, y_c, s_c )
=
\left(
\begin{array}{c}
g + G x - y_a + y_b + y_c^T C         \\
a + s_a - x                           \\
x + s_b - b                           \\
C x + c + s_c                         \\
D(s_a) D(y_a) 1_m                     \\
D(s_b) D(y_b) 1_m                     \\
D(s_c) D(y_c) 1_m
\end{array}
\right)
\] $$
where
$latex x \in \B{R}^n$$,
$latex y_a, s_a \in \B{R}_+^n$$,
$latex y_b, s_b \in \B{R}_+^n$$ and
$latex y_c, s_c \in \B{R}_+^m$$.
The KKT conditions for a solution of this problem is
$latex \[
    F ( x , y_a, s_a, y_b, s_b, y_c, s_c ) = 0
\] $$

$children%example/abs_normal/qp_box.cpp
    %example/abs_normal/qp_box.omh
%$$
$head Example$$
The file $cref qp_box.cpp$$ contains an example and test of
$code qp_box$$.

$end
-----------------------------------------------------------------------------
*/
# include "qp_interior.hpp"

// BEGIN C++
namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// BEGIN PROTOTYPE
template <class Vector>
bool qp_box(
    size_t        level   ,
    const Vector& a       ,
    const Vector& b       ,
    const Vector& c       ,
    const Vector& C       ,
    const Vector& g       ,
    const Vector& G       ,
    double        epsilon ,
    size_t        maxitr  ,
    const Vector& xin     ,
    Vector&       xout    )
// END PROTOTYPE
{   double inf = std::numeric_limits<double>::infinity();
    //
    size_t n = a.size();
    size_t m = c.size();
    //
    CPPAD_ASSERT_KNOWN(level <= 2, "qp_interior: level is greater than 2");
    CPPAD_ASSERT_KNOWN(
        size_t(b.size()) == n, "qp_box: size of b is not n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(C.size()) == m * n, "qp_box: size of C is not m * n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(g.size()) == n, "qp_box: size of g is not n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(G.size()) == n * n, "qp_box: size of G is not n * n"
    );
    if( level > 0 )
    {   std::cout << "start qp_box\n";
        CppAD::abs_print_mat("a", n, 1, a);
        CppAD::abs_print_mat("b", n, 1, b);
        CppAD::abs_print_mat("c", m, 1, c);
        CppAD::abs_print_mat("C", m, n, C);
        CppAD::abs_print_mat("g", 1, n, g);
        CppAD::abs_print_mat("G", n, n, G);
        CppAD::abs_print_mat("xin", n, 1, xin);
    }
    //
    // count number of lower and upper limits
    size_t n_limit = 0;
    for(size_t j = 0; j < n; j++)
    {   CPPAD_ASSERT_KNOWN(G[j * n + j] >= 0.0, "qp_box: G_{j,j} < 0.0");
        if( -inf < a[j] )
            ++n_limit;
        if( b[j] < inf )
            ++n_limit;
    }
    //
    // C_int and c_int define the extended constraints
    Vector C_int((m + n_limit) * n ), c_int(m + n_limit);
    for(size_t i = 0; i < size_t(C_int.size()); i++)
        C_int[i] = 0.0;
    //
    // put C * x + c <= 0 in C_int, c_int
    for(size_t i = 0; i < m; i++)
    {   c_int[i] = c[i];
        for(size_t j = 0; j < n; j++)
            C_int[i * n + j] = C[i * n + j];
    }
    //
    // put I * x - b <= 0 in C_int, c_int
    size_t i_limit = 0;
    for(size_t j = 0; j < n; j++) if( b[j] < inf )
    {   c_int[m + i_limit]            = - b[j];
        C_int[(m + i_limit) * n + j]  = 1.0;
        ++i_limit;
    }
    //
    // put a - I * x <= 0 in C_int, c_int
    for(size_t j = 0; j < n; j++) if( -inf < a[j] )
    {   c_int[m + i_limit]           = a[j];
        C_int[(m + i_limit) * n + j] = -1.0;
        ++i_limit;
    }
    Vector yout(m + n_limit), sout(m + n_limit);
    size_t level_int = 0;
    if( level == 2 )
        level_int = 1;
    bool ok = qp_interior( level_int,
        c_int, C_int, g, G, epsilon, maxitr, xin, xout, yout, sout
    );
    if( level > 0 )
    {   if( level < 2 )
            CppAD::abs_print_mat("xout", n, 1, xout);
        if( ok )
            std::cout << "end q_box: ok = true\n";
        else
            std::cout << "end q_box: ok = false\n";
    }
    return ok;
}

} // END_CPPAD_NAMESPACE
// END C++

# endif
