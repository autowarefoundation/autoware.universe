# ifndef CPPAD_CORE_ATOMIC_TWO_REVERSE_HPP
# define CPPAD_CORE_ATOMIC_TWO_REVERSE_HPP
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
$begin atomic_two_reverse$$
$spell
    sq
    mul.hpp
    afun
    ty
    px
    py
    Taylor
    const
    CppAD
    atx
    aty
    apx
    apy
    af
$$

$section Atomic Reverse Mode$$
$spell
    bool
$$

$head Syntax$$

$subhead Base$$
$icode%ok% = %afun%.reverse(%q%, %tx%, %ty%, %px%, %py%)
%$$
This syntax is used by $icode%f%.Forward%$$ where $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
and $icode afun$$ is used in $icode f$$.

$subhead AD<Base>$$
$icode%ok% = %afun%.reverse(%q%, %atx%, %aty%, %apx%, %apy%)
%$$
This syntax is used by $icode%af%.Forward%$$ where $icode af$$ has prototype
$codei%
    ADFun< AD<%Base%> , %Base% > %af%
%$$
and $icode afun$$ is used in $icode af$$ (see $cref base2ad$$).

$head Purpose$$
This function is used by $cref/reverse/Reverse/$$
to compute derivatives.

$head Implementation$$
If you are using
$cref/reverse/Reverse/$$ mode,
this virtual function must be defined by the
$cref/atomic_user/atomic_two_ctor/atomic_user/$$ class.
It can just return $icode%ok% == false%$$
(and not compute anything) for values
of $icode q$$ that are greater than those used by your
$cref/reverse/Reverse/$$ mode calculations.

$head q$$
The argument $icode q$$ has prototype
$codei%
    size_t %q%
%$$
It specifies the highest order Taylor coefficient that
computing the derivative of.

$head tx$$
The argument $icode tx$$ has prototype
$codei%
    const CppAD::vector<%Base%>& %tx%
%$$
and $icode%tx%.size() == (%q%+1)*%n%$$.
For $latex j = 0 , \ldots , n-1$$ and $latex k = 0 , \ldots , q$$,
we use the Taylor coefficient notation
$latex \[
\begin{array}{rcl}
    x_j^k    & = & tx [ j * ( q + 1 ) + k ]
    \\
    X_j (t)  & = & x_j^0 + x_j^1 t^1 + \cdots + x_j^q t^q
\end{array}
\] $$
Note that superscripts represent an index for $latex x_j^k$$
and an exponent for $latex t^k$$.
Also note that the Taylor coefficients for $latex X(t)$$ correspond
to the derivatives of $latex X(t)$$ at $latex t = 0$$ in the following way:
$latex \[
    x_j^k = \frac{1}{ k ! } X_j^{(k)} (0)
\] $$

$head atx$$
The argument $icode atx$$ has prototype
$codei%
    const CppAD::vector< AD<%Base%> >& %atx%
%$$
Otherwise, $icode atx$$ specifications are the same as for $icode tx$$.

$head ty$$
The argument $icode ty$$ has prototype
$codei%
    const CppAD::vector<%Base%>& %ty%
%$$
and $icode%tx%.size() == (%q%+1)*%m%$$.
For $latex i = 0 , \ldots , m-1$$ and $latex k = 0 , \ldots , q$$,
we use the Taylor coefficient notation
$latex \[
\begin{array}{rcl}
    Y_i (t)  & = & f_i [ X(t) ]
    \\
    Y_i (t)  & = & y_i^0 + y_i^1 t^1 + \cdots + y_i^q t^q + o ( t^q )
    \\
    y_i^k    & = & ty [ i * ( q + 1 ) + k ]
\end{array}
\] $$
where $latex o( t^q ) / t^q \rightarrow 0$$ as $latex t \rightarrow 0$$.
Note that superscripts represent an index for $latex y_j^k$$
and an exponent for $latex t^k$$.
Also note that the Taylor coefficients for $latex Y(t)$$ correspond
to the derivatives of $latex Y(t)$$ at $latex t = 0$$ in the following way:
$latex \[
    y_j^k = \frac{1}{ k ! } Y_j^{(k)} (0)
\] $$

$head aty$$
The argument $icode aty$$ has prototype
$codei%
    const CppAD::vector< AD<%Base%> >& %aty%
%$$
Otherwise, $icode aty$$ specifications are the same as for $icode ty$$.


$head F$$
We use the notation $latex \{ x_j^k \} \in \B{R}^{n \times (q+1)}$$ for
$latex \[
    \{ x_j^k \W{:} j = 0 , \ldots , n-1, k = 0 , \ldots , q \}
\]$$
We use the notation $latex \{ y_i^k \} \in \B{R}^{m \times (q+1)}$$ for
$latex \[
    \{ y_i^k \W{:} i = 0 , \ldots , m-1, k = 0 , \ldots , q \}
\]$$
We define the function
$latex F : \B{R}^{n \times (q+1)} \rightarrow \B{R}^{m \times (q+1)}$$ by
$latex \[
    y_i^k = F_i^k [ \{ x_j^k \} ]
\] $$
Note that
$latex \[
    F_i^0 ( \{ x_j^k \} ) = f_i ( X(0) )  = f_i ( x^0 )
\] $$
We also note that
$latex F_i^\ell ( \{ x_j^k \} )$$ is a function of
$latex x^0 , \ldots , x^\ell$$
and is determined by the derivatives of $latex f_i (x)$$
up to order $latex \ell$$.


$head G, H$$
We use $latex G : \B{R}^{m \times (q+1)} \rightarrow \B{R}$$
to denote an arbitrary scalar valued function of $latex \{ y_i^k \}$$.
We use $latex H : \B{R}^{n \times (q+1)} \rightarrow \B{R}$$
defined by
$latex \[
    H ( \{ x_j^k \} ) = G[ F( \{ x_j^k \} ) ]
\] $$

$head py$$
The argument $icode py$$ has prototype
$codei%
    const CppAD::vector<%Base%>& %py%
%$$
and $icode%py%.size() == m * (%q%+1)%$$.
For $latex i = 0 , \ldots , m-1$$, $latex k = 0 , \ldots , q$$,
$latex \[
    py[ i * (q + 1 ) + k ] = \partial G / \partial y_i^k
\] $$

$head apy$$
The argument $icode apy$$ has prototype
$codei%
    const CppAD::vector< AD<%Base%> >& %apy%
%$$
Otherwise, $icode apy$$ specifications are the same as for $icode py$$.

$subhead px$$
The $icode px$$ has prototype
$codei%
    CppAD::vector<%Base%>& %px%
%$$
and $icode%px%.size() == n * (%q%+1)%$$.
The input values of the elements of $icode px$$
are not specified (must not matter).
Upon return,
for $latex j = 0 , \ldots , n-1$$ and $latex \ell = 0 , \ldots , q$$,
$latex \[
\begin{array}{rcl}
px [ j * (q + 1) + \ell ] & = & \partial H / \partial x_j^\ell
\\
& = &
( \partial G / \partial \{ y_i^k \} ) \cdot
    ( \partial \{ y_i^k \} / \partial x_j^\ell )
\\
& = &
\sum_{k=0}^q
\sum_{i=0}^{m-1}
( \partial G / \partial y_i^k ) ( \partial y_i^k / \partial x_j^\ell )
\\
& = &
\sum_{k=\ell}^q
\sum_{i=0}^{m-1}
py[ i * (q + 1 ) + k ] ( \partial F_i^k / \partial x_j^\ell )
\end{array}
\] $$
Note that we have used the fact that for $latex k < \ell$$,
$latex \partial F_i^k / \partial x_j^\ell = 0$$.

$head apx$$
The argument $icode apx$$ has prototype
$codei%
    CppAD::vector< AD<%Base%> >& %apx%
%$$
Otherwise, $icode apx$$ specifications are the same as for $icode px$$.

$head ok$$
The return value $icode ok$$ has prototype
$codei%
    bool %ok%
%$$
If it is $code true$$, the corresponding evaluation succeeded,
otherwise it failed.

$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/two_reverse.hpp
Atomic reverse mode.
*/
/*!
Link from reverse mode sweep to users routine.

\param q [in]
highest order for this reverse mode calculation.

\param tx [in]
Taylor coefficients corresponding to x for this calculation.

\param ty [in]
Taylor coefficient corresponding to y for this calculation

\param px [out]
Partials w.r.t. the x Taylor coefficients.

\param py [in]
Partials w.r.t. the y Taylor coefficients.

See atomic_reverse mode use documentation
*/
template <class Base>
bool atomic_base<Base>::reverse(
    size_t                    q  ,
    const vector<Base>&       tx ,
    const vector<Base>&       ty ,
          vector<Base>&       px ,
    const vector<Base>&       py )
{   return false; }

template <class Base>
bool atomic_base<Base>::reverse(
    size_t                    q  ,
    const vector< AD<Base> >& atx ,
    const vector< AD<Base> >& aty ,
          vector< AD<Base> >& apx ,
    const vector< AD<Base> >& apy )
{   return false; }

} // END_CPPAD_NAMESPACE
# endif
