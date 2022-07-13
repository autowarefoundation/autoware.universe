# ifndef CPPAD_CORE_ATOMIC_TWO_CTOR_HPP
# define CPPAD_CORE_ATOMIC_TWO_CTOR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_two_ctor$$
$spell
    enum
    sq
    std
    afun
    arg
    CppAD
    bool
    ctor
    const
    mat_mul_xam.cpp
    hpp
$$

$section Atomic Function Constructor$$

$head Syntax$$
$icode%atomic_user afun%(%ctor_arg_list%)
%$$
$codei%atomic_base<%Base%>(%name%, %sparsity%)
%$$

$head atomic_user$$

$subhead ctor_arg_list$$
Is a list of arguments for the $icode atomic_user$$ constructor.

$subhead afun$$
The object $icode afun$$ must stay in scope for as long
as the corresponding atomic function is used.
This includes use by any $cref/ADFun<Base>/ADFun/$$ that
has this $icode atomic_user$$ operation in its
$cref/operation sequence/glossary/Operation/Sequence/$$.

$subhead Implementation$$
The user defined $icode atomic_user$$ class is a publicly derived class of
$codei%atomic_base<%Base%>%$$.
It should be declared as follows:
$codei%
    class %atomic_user% : public CppAD::atomic_base<%Base%> {
    public:
        %atomic_user%(%ctor_arg_list%) : atomic_base<%Base%>(%name%, %sparsity%)
    %...%
    };
%$$
where $icode ...$$
denotes the rest of the implementation of the derived class.
This includes completing the constructor and
all the virtual functions that have their
$code atomic_base$$ implementations replaced by
$icode atomic_user$$ implementations.

$head atomic_base$$

$subhead Restrictions$$
The $code atomic_base$$ constructor and destructor cannot be called in
$cref/parallel/ta_in_parallel/$$ mode.

$subhead Base$$
The template parameter determines the
$icode Base$$ type for this $codei%AD<%Base%>%$$ atomic operation.

$subhead name$$
This $code atomic_base$$ constructor argument has the following prototype
$codei%
    const std::string& %name%
%$$
It is the name for this atomic function and is used for error reporting.
The suggested value for $icode name$$ is $icode afun$$ or $icode atomic_user$$,
i.e., the name of the corresponding atomic object or class.

$subhead sparsity$$
This $code atomic_base$$ constructor argument has prototype
$codei%
    atomic_base<%Base%>::option_enum %sparsity%
%$$
The current $icode sparsity$$ for an $code atomic_base$$ object
determines which type of sparsity patterns it uses
and its value is one of the following:
$table
$icode sparsity$$   $cnext sparsity patterns $rnext
$codei%atomic_base<%Base%>::pack_sparsity_enum%$$ $pre  $$ $cnext
    $cref/vectorBool/CppAD_vector/vectorBool/$$
$rnext
$codei%atomic_base<%Base%>::bool_sparsity_enum%$$ $pre  $$ $cnext
    $cref/vector/CppAD_vector/$$$code <bool>$$
$rnext
$codei%atomic_base<%Base%>::set_sparsity_enum%$$ $pre  $$ $cnext
    $cref/vector/CppAD_vector/$$$code <std::set<std::size_t> >$$
$tend
There is a default value for $icode sparsity$$ if it is not
included in the constructor (which may be either the bool or set option).

$end
-------------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/two_ctor.hpp
Constructors for atomic_base class.
*/

/*!
Base class for atomic_atomic functions.

\tparam Base
This class is used for defining an AD<Base> atomic operation y = f(x).

\par
make sure user does not invoke the default constructor
*/
template <class Base>
atomic_base<Base>::atomic_base(void)
{   CPPAD_ASSERT_KNOWN(false,
        "Attempt to use the atomic_base default constructor"
    );
}
/*!
Constructor

\param name
name used for error reporting

\param sparsity [in]
what type of sparsity patterns are computed by this function,
bool_sparsity_enum or set_sparsity_enum. Default value is
bool sparsity patterns.
*/
template <class Base>
atomic_base<Base>::atomic_base(
        const std::string&     name,
        option_enum            sparsity
) :
sparsity_( sparsity               )
{   CPPAD_ASSERT_KNOWN(
        ! thread_alloc::in_parallel() ,
        "atomic_base: constructor cannot be called in parallel mode."
    );
    CPPAD_ASSERT_UNKNOWN( constant_enum < dynamic_enum );
    CPPAD_ASSERT_UNKNOWN( dynamic_enum < variable_enum );
    //
    // atomic_index
    bool        set_null  = false;
    size_t      index     = 0;
    size_t      type      = 2;
    std::string copy_name = name;
    void*       copy_this = reinterpret_cast<void*>( this );
    index_  = local::atomic_index<Base>(
        set_null, index, type, &copy_name, copy_this
    );
    // initialize work pointers as null;
    for(size_t thread = 0; thread < CPPAD_MAX_NUM_THREADS; thread++)
        work_[thread] = nullptr;
}

} // END_CPPAD_NAMESPACE
# endif
