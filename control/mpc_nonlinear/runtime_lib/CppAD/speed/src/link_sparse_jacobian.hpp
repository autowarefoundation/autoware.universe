# ifndef CPPAD_SPEED_SRC_LINK_SPARSE_JACOBIAN_HPP
# define CPPAD_SPEED_SRC_LINK_SPARSE_JACOBIAN_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/utility/vector.hpp>
// BEGIN PROTOTYPE
extern bool link_sparse_jacobian(
    const std::string&                job       ,
    size_t                            size      ,
    size_t                            repeat    ,
    size_t                            m         ,
    const CppAD::vector<size_t>&      row       ,
    const CppAD::vector<size_t>&      col       ,
          CppAD::vector<double>&      x         ,
          CppAD::vector<double>&      jacobian  ,
          size_t&                     n_color
);
// END PROTOTYPE
/*
------------------------------------------------------------------------------
$begin link_sparse_jacobian$$
$spell
    colpack
    cppad
    const
    bool
    CppAD
    Jacobian
    Jacobians
$$


$section Speed Testing Sparse Jacobians$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%0
%$$

$head Method$$
Given a range space dimension $icode m$$
the row index vector $latex row$$, and column index vector $latex col$$,
a corresponding function $latex f : \B{R}^n \rightarrow \B{R}^m $$
is defined by $cref sparse_jac_fun$$.
The non-zero entries in the Jacobian of this function have the form
$latex \[
    \D{f[row[k]]}{x[col[k]]]}
\] $$
for some $latex k$$ between zero and $icode%K% = %row%.size()-1%$$.
All the other terms of the Jacobian are zero.

$head Sparsity Pattern$$
The combination of $icode row$$ and $icode col$$ determine
the sparsity pattern for the Jacobian that is differentiated.
The calculation of this sparsity pattern,
if necessary to compute the Jacobian,
is intended to be part of the timing for this test.

$head job$$
See the standard link specifications for $cref/job/link_routines/job/$$.

$head size$$
See the standard link specifications for $cref/size/link_routines/size/$$.
In addition, $icode size$$ is referred to as $latex n$$ below,
is the dimension of the domain space for $latex f(x)$$.

$head repeat$$
See the standard link specifications for $cref/repeat/link_routines/repeat/$$.

$head m$$
Is the dimension of the range space for the function $latex f(x)$$.

$head row$$
The size of the vector $icode row$$ defines the value $latex K$$.
The input value of its elements does not matter.
On output,
all the elements of $icode row$$ are between zero and $latex m-1$$.

$head col$$
The argument $icode col$$ is a vector with size $latex K$$.
The input value of its elements does not matter.
On output,
all the elements of $icode col$$ are between zero and $latex n-1$$.

$head Row Major$$
The indices $icode row$$ and $icode col$$ are in row major order; i.e.,
for each $icode%k% < %row%.size()-2%$$
$codei%
    %row%[%k%] <= %row%[%k%+1]
%$$
and if $icode%row%[%k%] == %row%[%k%+1]%$$ then
$codei%
    %col%[%k%] < %col%[%k%+1]
%$$

$head x$$
The argument $icode x$$ has prototype
$codei%
   CppAD::vector<double>& %x%
%$$
and its size is $latex n$$; i.e., $icode%x%.size() == %size%$$.
The input value of the elements of $icode x$$ does not matter.
On output, it has been set to the
argument value for which the function,
or its derivative, is being evaluated and placed in $icode jacobian$$.
The value of this vector need not change with each repetition.

$head jacobian$$
The argument $icode jacobian$$ has prototype
$codei%
   CppAD::vector<double>& %jacobian%
%$$
and its size is $icode K$$.
The input value of its elements does not matter.
The output value of its elements is the Jacobian of the function $latex f(x)$$.
To be more specific, for
$latex k = 0 , \ldots , K - 1$$,
$latex \[
    \D{f[ \R{row}[k] ]}{x[ \R{col}[k] ]} (x) = \R{jacobian} [k]
\] $$

$head n_color$$
The input value of $icode n_color$$ does not matter. On output,
it has value zero or $cref/n_sweep/sparse_jacobian/n_sweep/$$ corresponding
to the evaluation of $icode jacobian$$.
This is also the number of colors corresponding to the
$cref/coloring method/sparse_jacobian/work/color_method/$$,
which can be set to $cref/colpack/speed_main/Sparsity Options/colpack/$$,
and is otherwise $code cppad$$.
If this routine returns an non-zero $icode n_color$$ for
any $icode job$$ value, the non-zero value will be reported for this test.

$subhead double$$
In the case where $icode package$$ is $code double$$,
only the first $latex m$$
elements of $icode jacobian$$ are used and they are set to
the value of $latex f(x)$$.

$end
-----------------------------------------------------------------------------
*/
# endif
