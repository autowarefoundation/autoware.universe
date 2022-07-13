# ifndef CPPAD_WNO_CONVERSION_HPP
# define CPPAD_WNO_CONVERSION_HPP
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
$begin wno_conversion$$
$spell
    cppad
    wno
    cpp
    hpp
$$

$section Suppress Suspect Implicit Conversion Warnings$$

$head Syntax$$
$codei%# include <cppad/wno_conversion.hpp>%$$

$head Purpose$$
In many cases it is good to have warnings for implicit conversions
that may loose range or precision.
The include command above, before any other includes, suppresses
these warning for a particular compilation unit (which usually corresponds
to a $icode%*%.cpp%$$ file).

$end
*/

# include <cppad/configure.hpp>
# if CPPAD_COMPILER_HAS_CONVERSION_WARN
# pragma GCC diagnostic ignored "-Wfloat-conversion"
# pragma GCC diagnostic ignored "-Wconversion"
# endif

# endif
