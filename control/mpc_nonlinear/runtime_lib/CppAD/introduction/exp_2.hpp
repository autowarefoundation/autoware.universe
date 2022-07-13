# ifndef CPPAD_INTRODUCTION_EXP_2_HPP
# define CPPAD_INTRODUCTION_EXP_2_HPP
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
$begin exp_2$$
$spell
    cppad-%yyyymmdd%
    hpp
    Apx
    cpp
    const
    exp
    bool
$$

$section Second Order Exponential Approximation$$


$head Syntax$$
$codei%# include "exp_2.hpp"%$$
$pre
$$
$icode%y% = exp_2(%x%)%$$


$head Purpose$$
This is a simple example algorithm that is used to demonstrate
Algorithmic Differentiation
(see $cref exp_eps$$ for a more complex example).

$head Mathematical Form$$
The exponential function can be defined by
$latex \[
    \exp (x) = 1 + x^1 / 1 ! + x^2 / 2 ! + \cdots
\] $$
The second order approximation for the exponential function is
$latex \[
{\rm exp\_2} (x) =  1 + x + x^2 / 2
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
approximation for the second order exponential approximation.

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
    $cnext construct object with value equal to $icode i$$
$rnext
$icode%Type u %=% v%$$
    $cnext $icode Type$$
    $cnext construct $icode u$$ with value equal to $icode v$$
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
$tend

$childtable%
    introduction/exp_2.omh%
    introduction/exp_2_cppad.cpp
%$$


$head Implementation$$
The file $cref exp_2.hpp$$
contains a C++ implementation of this function.

$head Test$$
The file $cref exp_2.cpp$$
contains a test of this implementation.


$head Exercises$$
$list number$$
Suppose that we make the call
$codep
    double x = .1;
    double y = exp_2(x);
$$
What is the value assigned to
$code v1$$, $code v2$$, ... ,$code v5$$ in $cref exp_2.hpp$$ ?
$lnext
Extend the routine $code exp_2.hpp$$ to
a routine $code exp_3.hpp$$ that computes
$latex \[
    1 + x^2 / 2 ! + x^3 / 3 !
\] $$
Do this in a way that only assigns one value to each variable
(as $code exp_2$$ does).
$lnext
Suppose that we make the call
$codep
    double x = .5;
    double y = exp_3(x);
$$
using $code exp_3$$ created in the previous problem.
What is the value assigned to the new variables in $code exp_3$$
(variables that are in $code exp_3$$ and not in $code exp_2$$) ?
$lend

$end
------------------------------------------------------------------------------
*/
// BEGIN C++
template <class Type>
Type exp_2(const Type &x)
{       Type v1  = x;                // v1 = x
        Type v2  = Type(1) + v1;     // v2 = 1 + x
        Type v3  = v1 * v1;          // v3 = x^2
        Type v4  = v3 / Type(2);     // v4 = x^2 / 2
        Type v5  = v2 + v4;          // v5 = 1 + x + x^2 / 2
        return v5;                   // exp_2(x) = 1 + x + x^2 / 2
}
// END C++

# endif
