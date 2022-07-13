# ifndef CPPAD_CORE_AD_TYPE_HPP
# define CPPAD_CORE_AD_TYPE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/is_pod.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*
$begin ad_type_enum$$
$spell
    enum
    typedef
    CppAD
    namespace
$$

$section Type of AD an Object$$

$head User API$$
The values $code constant_enum$$, $code dynamic_enum$$ and
$code variable_enum$$ are in the user API; see
$cref/ad_type/atomic_three/ad_type/$$ for $code atomic_three$$ functions.

$head typedef$$
This typedef is in the $code CppAD$$ namespace:
$srccode%hpp% */
    typedef enum {
        constant_enum,            // constant parameter
        dynamic_enum,             // dynamic parameter
        variable_enum,            // variable
        number_ad_type_enum       // number of valid values for type_ad_enum
    } ad_type_enum;
/* %$$

$head is_pod$$
The following informs $cref is_pod$$ that this is plain old data.
$srccode%hpp% */
    namespace local {
        template <> inline bool
        is_pod<ad_type_enum>(void) { return true; }
    }
/* %$$
$end
*/

} // END_CPPAD_NAMESPACE


# endif
