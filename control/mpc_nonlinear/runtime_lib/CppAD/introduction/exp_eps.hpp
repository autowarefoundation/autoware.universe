# ifndef CPPAD_INTRODUCTION_EXP_EPS_HPP
# define CPPAD_INTRODUCTION_EXP_EPS_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin exp_eps$$
$spell
    cppad-%yyyymmdd%
    hpp
    Apx
    cpp
    const
    exp_eps
    bool
$$

$section An Epsilon Accurate Exponential Approximation$$


$head Syntax$$
$codei%# include "exp_eps.hpp"%$$
$pre
$$
$icode%y% = exp_eps(%x%, %epsilon%)%$$


$head Purpose$$
This is a an example algorithm that is used to demonstrate
how Algorithmic Differentiation works with loops and
boolean decision variables
(see $cref exp_2$$ for a simpler example).

$head Mathematical Function$$
The exponential function can be defined by
$latex \[
    \exp (x) = 1 + x^1 / 1 ! + x^2 / 2 ! + \cdots
\] $$
We define $latex k ( x, \varepsilon )  $$ as the smallest
non-negative integer such that $latex \varepsilon \geq x^k / k !$$; i.e.,
$latex \[
k( x, \varepsilon ) =
    \min \{ k \in {\rm Z}_+ \; | \; \varepsilon \geq x^k / k ! \}
\] $$
The mathematical form for our approximation of the exponential function is
$latex \[
\begin{array}{rcl}
{\rm exp\_eps} (x , \varepsilon ) & = & \left\{
\begin{array}{ll}
\frac{1}{ {\rm exp\_eps} (-x , \varepsilon ) }
    & {\rm if} \; x < 0
\\
1 + x^1 / 1 ! + \cdots + x^{k( x, \varepsilon)} / k( x, \varepsilon ) !
    & {\rm otherwise}
\end{array}
\right.
\end{array}
\] $$


$head include$$
The include command in the syntax is relative to
$codei%
    cppad-%yyyymmdd%/introduction/exp_apx
%$$
where $codei%cppad-%yyyymmdd%$$ is the distribution directory
created during the beginning steps of the
$cref%installation%Install%$$ of CppAD.


$head x$$
The argument $icode x$$ has prototype
$codei%
    const %Type% &%x%
%$$
(see $icode Type$$ below).
It specifies the point at which to evaluate the
approximation for the exponential function.

$head epsilon$$
The argument $icode epsilon$$ has prototype
$codei%
    const %Type% &%epsilon%
%$$
It specifies the accuracy with which
to approximate the exponential function value; i.e.,
it is the value of $latex \varepsilon$$ in the
exponential function approximation defined above.

$head y$$
The result $icode y$$ has prototype
$codei%
    %Type% %y%
%$$
It is the value of the exponential function
approximation defined above.

$head Type$$
If $icode u$$ and $icode v$$ are $icode Type$$ objects and $icode i$$
is an $code int$$:

$table
$bold Operation$$  $cnext $bold Result Type$$ $cnext $bold Description$$
$rnext
$icode%Type%(%i%)%$$
    $cnext $icode Type$$
    $cnext object with value equal to $icode i$$
$rnext
$icode%Type u %=% v%$$
    $cnext $icode Type$$
    $cnext construct $icode u$$ with value equal to $icode v$$
$rnext
$icode%u% > %v%$$
    $cnext $code bool$$
    $cnext true,
    if $icode u$$ greater than $icode v$$, an false otherwise
$rnext
$icode%u% = %v%$$
    $cnext $icode Type$$
    $cnext new $icode u$$ (and result) is value of $icode v$$
$rnext
$icode%u% * %v%$$
    $cnext $icode Type$$
    $cnext result is value of $latex u * v$$
$rnext
$icode%u% / %v%$$
    $cnext $icode Type$$
    $cnext result is value of $latex u / v$$
$rnext
$icode%u% + %v%$$
    $cnext $icode Type$$
    $cnext result is value of $latex u + v$$
$rnext
$codei%-%u%$$
    $cnext $icode Type$$
    $cnext result is value of $latex - u$$
$tend

$children%
    introduction/exp_eps.omh%
    introduction/exp_eps_cppad.cpp
%$$

$head Implementation$$
The file $cref exp_eps.hpp$$
contains a C++ implementation of this function.

$head Test$$
The file $cref exp_eps.cpp$$
contains a test of this implementation.

$head Exercises$$
$list number$$
Using the definition of $latex k( x, \varepsilon )$$ above,
what is the value of
$latex k(.5, 1)$$, $latex k(.5, .1)$$, and $latex k(.5, .01)$$ ?
$lnext
Suppose that we make the following call to $code exp_eps$$:
$codep
    double x       = 1.;
    double epsilon = .01;
    double y = exp_eps(x, epsilon);
$$
What is the value assigned to
$code k$$, $code temp$$, $code term$$, and $code sum$$
the first time through the $code while$$ loop in $cref exp_eps.hpp$$ ?
$lnext
Continuing the previous exercise,
what is the value assigned to
$code k$$, $code temp$$, $code term$$, and $code sum$$
the second time through the $code while$$ loop in $cref exp_eps.hpp$$ ?
$lend


$end
-----------------------------------------------------------------------------
*/
// BEGIN C++
template <class Type>
Type exp_eps(const Type &x, const Type &epsilon)
{   // abs_x = |x|
    Type abs_x = x;
    if( Type(0) > x )
        abs_x = - x;
    // initialize
    int  k    = 0;          // initial order
    Type term = 1.;         // term = |x|^k / k !
    Type sum  = term;       // initial sum
    while(term > epsilon)
    {   k         = k + 1;          // order for next term
        Type temp = term * abs_x;   // term = |x|^k / (k-1)!
        term      = temp / Type(k); // term = |x|^k / k !
        sum       = sum + term;     // sum  = 1 + ... + |x|^k / k !
    }
    // In the case where x is negative, use exp(x) = 1 / exp(-|x|)
    if( Type(0) > x )
        sum = Type(1) / sum;
    return sum;
}
// END C++

# endif
