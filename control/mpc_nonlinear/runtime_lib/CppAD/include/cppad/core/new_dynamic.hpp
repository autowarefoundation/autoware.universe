# ifndef CPPAD_CORE_NEW_DYNAMIC_HPP
# define CPPAD_CORE_NEW_DYNAMIC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin new_dynamic$$
$spell
    const
    Taylor
    cpp
    dyn
    ind
$$

$section Change the Dynamic Parameters$$

$head Syntax$$
$icode%f%.new_dynamic(%dynamic%)%$$

$head Purpose$$
Often one is only interested in computing derivatives with respect
to a subset of arguments to a function.
In this case, it is easier to make all the arguments to the function
$cref/independent variables/glossary/Tape/Independent Variable/$$.
It is more efficient,
will use less memory and be faster,
if the only the argument were are computing derivatives with respect to
are independent variables and the other arguments are
$cref/dynamic/glossary/Parameter/Dynamic/$$ parameters.
The $code new_dynamic$$ method is used to change the value
of the dynamic parameters in $icode f$$.

$head f$$
The object $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
Note that the $cref ADFun$$ object $icode f$$ is not $code const$$.

$head dynamic$$
This argument has prototype
$codei%
    const %BaseVector%& %dynamic%
%$$
(see $icode BaseVector$$ below).
It specifies a new value for the independent
$cref/dynamic/glossary/Parameter/Dynamic/$$ parameters.
It size must be the same as the size of the independent
$cref/dynamic/Independent/dynamic/$$ parameter vector
in the call to $code Independent$$ that started
the recording for $icode f$$; see
$cref/size_dyn_ind/fun_property/size_dyn_ind/$$.

$head BaseVector$$
The type $icode BaseVector$$ must be a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$icode Base$$.

$head Taylor Coefficients$$
The Taylor coefficients computed by previous calls to
$cref/f.Forward/Forward/$$ are lost after this operation; including the
order zero coefficients (because they may depend on the dynamic parameters).
In order words;
$cref/f.size_order/size_order/$$ returns zero directly after
$icode%f%.new_dynamic%$$ is called.

$children%
    example/general/new_dynamic.cpp
%$$
$head Example$$
The file $cref new_dynamic.cpp$$
contains an example and test of this operation.

$end
*/
# include <cppad/local/sweep/dynamic.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file new_dynamic.hpp
User interface to ADFun dynamic_parameter member function.
*/

/*!
Change the dynamic parameters in this ADFun object

\param dynamic
is the vector of new values for the dynamic parameters.
*/
template <class Base, class RecBase>
template <class BaseVector>
void ADFun<Base,RecBase>::new_dynamic(const BaseVector& dynamic)
{   using local::pod_vector;
    CPPAD_ASSERT_KNOWN(
        size_t( dynamic.size() ) == play_.num_dynamic_ind() ,
        "f.new_dynamic: dynamic.size() different from corresponding "
        "call to Independent"
    );
    // check BaseVector is Simple Vector class with Base elements
    CheckSimpleVector<Base, BaseVector>();

    // retrieve player information about the dynamic parameters
    local::pod_vector_maybe<Base>&     all_par_vec( play_.all_par_vec() );
    const pod_vector<bool>&            dyn_par_is ( play_.dyn_par_is()  );
    const pod_vector<local::opcode_t>& dyn_par_op ( play_.dyn_par_op()  );
    const pod_vector<addr_t>&          dyn_par_arg( play_.dyn_par_arg() );
    const pod_vector<addr_t>&     dyn_ind2par_ind ( play_.dyn_ind2par_ind() );

    // set the dependent dynamic parameters
    RecBase not_used_rec_base(0.0);
    local::sweep::dynamic(
        all_par_vec         ,
        dynamic             ,
        dyn_par_is          ,
        dyn_ind2par_ind     ,
        dyn_par_op          ,
        dyn_par_arg         ,
        not_used_rec_base
    );

    // the existing Taylor coefficients are no longer valid
    num_order_taylor_ = 0;

    return;
}


} // END_CPPAD_NAMESPACE
# endif
