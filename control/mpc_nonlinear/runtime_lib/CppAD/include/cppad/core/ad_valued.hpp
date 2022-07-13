# ifndef CPPAD_CORE_AD_VALUED_HPP
# define CPPAD_CORE_AD_VALUED_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin ADValued$$
$spell
$$


$section AD Valued Operations and Functions$$

$comment atomic.omh includes atomic_two.hpp$$
$childtable%
    include/cppad/core/arithmetic.hpp%
    include/cppad/core/standard_math.hpp%
    include/cppad/core/cond_exp.hpp%
    include/cppad/core/discrete/user.omh%
    include/cppad/core/numeric_limits.hpp%
    include/cppad/core/atomic/atomic.omh
%$$

$end
*/

// include MathOther.h after CondExp.h because some MathOther.h routines use
// CondExp.h and CondExp.h is not sufficently declared in Declare.h

# include <cppad/core/arithmetic.hpp>
# include <cppad/core/standard_math.hpp>
# include <cppad/core/azmul.hpp>
# include <cppad/core/cond_exp.hpp>
# include <cppad/core/discrete/discrete.hpp>
# include <cppad/core/atomic/atomic_three.hpp>
# include <cppad/core/chkpoint_two/chkpoint_two.hpp>
# include <cppad/core/atomic/atomic_two.hpp>
# include <cppad/core/atomic/atomic_one.hpp>
# include <cppad/core/chkpoint_one/chkpoint_one.hpp>

# endif
