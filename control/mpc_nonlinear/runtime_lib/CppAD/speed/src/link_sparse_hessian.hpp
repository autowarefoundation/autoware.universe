# ifndef CPPAD_SPEED_SRC_LINK_SPARSE_HESSIAN_HPP
# define CPPAD_SPEED_SRC_LINK_SPARSE_HESSIAN_HPP

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
extern bool link_sparse_hessian(
    size_t                           size      ,
    size_t                           repeat    ,
    const CppAD::vector<size_t>&     row       ,
    const CppAD::vector<size_t>&     col       ,
    CppAD::vector<double>&           x         ,
    CppAD::vector<double>&           hessian   ,
    size_t&                          n_color
);
// END PROTOTYPE
/*
-------------------------------------------------------------------------------
$begin link_sparse_hessian$$
$spell
    const
    bool
    CppAD
    cppad
    colpack
    namespace
$$


$section Link to Speed Test Sparse Hessian$$

$head Syntax$$
$icode%ok% = link_sparse_hessian(
        %size%, %repeat%, %row%, %col%, %x%, %hessian%, %n_color%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%0
%$$

$head Namespace$$
The function $code link_sparse_hessian$$ is in the global namespace,
not the $code CppAD$$ namespace.

$head Method$$
Given a row index vector $latex row$$
and a second column vector $latex col$$,
the corresponding function
$latex f : \B{R}^n \rightarrow \B{R} $$
is defined by $cref sparse_hes_fun$$.
The non-zero entries in the Hessian of this function have
one of the following forms:
$latex \[
    \DD{f}{x[row[k]]}{x[row[k]]}
    \; , \;
    \DD{f}{x[row[k]]}{x[col[k]]}
    \; , \;
    \DD{f}{x[col[k]]}{x[row[k]]}
    \; , \;
    \DD{f}{x[col[k]]}{x[col[k]]}
\] $$
for some $latex k $$ between zero and $latex K-1 $$.
All the other terms of the Hessian are zero.

$head Sparsity Pattern$$
The combination of $icode row$$ and $icode col$$ determine
the sparsity pattern for the Hessian that is computed.
The calculation of this sparsity pattern,
if necessary to compute the Hessian,
is intended to be part of the timing for this test.

$head size$$
The argument $icode size$$, referred to as $latex n$$ below,
is the dimension of the domain space for $latex f(x)$$.

$head repeat$$
The argument $icode repeat$$ is the number of times
to repeat the test
(with a different value for $icode x$$ corresponding to
each repetition).

$head x$$
The size of $icode x$$ is $latex n$$; i.e., $icode%x%.size() == %size%$$.
The input value of the elements of $icode x$$ does not matter.
On output, it has been set to the
argument value for which the function,
or its derivative, is being evaluated.
The value of this vector need not change with each repetition.

$head row$$
The size of the vector $icode row$$ defines the value $latex K$$.
The input value of its elements does not matter.
On output,
all the elements of $icode row$$ are between zero and $latex n-1$$.

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

$head Lower Triangular$$
Only the lower triangle of the Hessian is included.
$codei%
    %col%[%k%] <= %row%[%k%]
%$$.


$head hessian$$
The size of $icode hessian$$ is $icode K$$.
The input value of its elements does not matter.
The output value of its elements is the Hessian of the function $latex f(x)$$.
To be more specific, for
$latex k = 0 , \ldots , K-1$$,
$latex \[
    \DD{f}{ x[ \R{row}[k] ] }{ x[ \R{col}[k] ]} = \R{hessian} [k]
\] $$

$head n_color$$
The input value of $icode n_color$$ does not matter. On output,
it is the value $cref/n_sweep/sparse_hessian/n_sweep/$$ corresponding
to the evaluation of $icode hessian$$.
This is also the number of colors corresponding to the
$cref/coloring method/sparse_hessian/work/color_method/$$,
which can be set to $cref/colpack/speed_main/Sparsity Options/colpack/$$,
and is otherwise $code cppad$$.


$subhead double$$
In the case where $icode package$$ is $code double$$,
only the first element of $icode hessian$$ is used and it is actually
the value of $latex f(x)$$ (derivatives are not computed).

$end
-----------------------------------------------------------------------------
*/

# endif
