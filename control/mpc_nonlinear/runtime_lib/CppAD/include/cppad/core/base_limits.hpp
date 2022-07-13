# ifndef CPPAD_CORE_BASE_LIMITS_HPP
# define CPPAD_CORE_BASE_LIMITS_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin base_limits$$
$spell
    std
    namespace
    CppAD
$$

$section Base Type Requirements for Numeric Limits$$

$head CppAD::numeric_limits$$
A specialization for
$cref/CppAD::numeric_limits/numeric_limits/$$
must be defined in order to use the type $codei%AD<%Base%>%$$.
CppAD does not use a specialization of
$codei%std::numeric_limits<%Base%>%$$.
Since C++11, using a specialization of
$codei%std::numeric_limits<%Base%>%$$
would require that $icode Base$$ be a literal type.

$head CPPAD_NUMERIC_LIMITS$$
In most cases, this macro can be used to define the specialization where
the numeric limits for the type $icode Base$$
are the same as the standard numeric limits for the type $icode Other$$.
For most $icode Base$$ types,
there is a choice of $icode Other$$,
for which the following preprocessor macro invocation suffices:
$codei%
    namespace CppAD {
        CPPAD_NUMERIC_LIMITS(%Other%, %Base%)
    }
%$$
where the macro is defined by
$srccode%cpp% */
# define CPPAD_NUMERIC_LIMITS(Other, Base) \
template <> class numeric_limits<Base>\
{\
    public:\
    static Base min(void) \
    {   return static_cast<Base>( std::numeric_limits<Other>::min() ); }\
    static Base max(void) \
    {   return static_cast<Base>( std::numeric_limits<Other>::max() ); }\
    static Base epsilon(void) \
    {   return static_cast<Base>( std::numeric_limits<Other>::epsilon() ); }\
    static Base quiet_NaN(void) \
    {   return static_cast<Base>( std::numeric_limits<Other>::quiet_NaN() ); }\
    static Base infinity(void) \
    {   return static_cast<Base>( std::numeric_limits<Other>::infinity() ); }\
    static const int digits10 = std::numeric_limits<Other>::digits10;\
};
/* %$$
$end
*/

# endif
