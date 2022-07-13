# ifndef CPPAD_CORE_ATOMIC_THREE_CTOR_HPP
# define CPPAD_CORE_ATOMIC_THREE_CTOR_HPP
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
$begin atomic_three_ctor$$
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
$codei%class %atomic_user% : public CppAD::atomic_three<%Base%> {
public:
    %atomic_user%(%ctor_arg_list%) : CppAD::atomic_three<%Base%>(%name%)
    %...%
};
%$$
$icode%atomic_user afun%(%ctor_arg_list%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
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
$codei%atomic_three<%Base%>%$$.
It should be declared as follows:
$codei%
    class %atomic_user% : public CppAD::atomic_three<%Base%> {
    public:
        %atomic_user%(%ctor_arg_list%) : atomic_three<%Base%>(%name%)
    %...%
    };
%$$
where $icode ...$$
denotes the rest of the implementation of the derived class.
This includes completing the constructor and
all the virtual functions that have their
$code atomic_three$$ implementations replaced by
$icode atomic_user$$ implementations.

$head atomic_three$$

$subhead Restrictions$$
The $code atomic_three$$ constructor and destructor cannot be called in
$cref/parallel/ta_in_parallel/$$ mode.

$subhead Base$$
The template parameter determines the
$cref/Base/atomic_three_afun/Base/$$
type for this $codei%AD<%Base%>%$$ atomic operation.

$subhead name$$
This $code atomic_three$$ constructor argument has the following prototype
$codei%
    const std::string& %name%
%$$
It is the name for this atomic function and is used for error reporting.
The suggested value for $icode name$$ is $icode afun$$ or $icode atomic_user$$,
i.e., the name of the corresponding atomic object or class.

$head Example$$

$subhead Define Constructor$$
The following is an example of a atomic function constructor definition:
$cref%get_started.cpp%atomic_three_get_started.cpp%Constructor%$$.

$subhead Use Constructor$$
The following is an example using a atomic function constructor:
$cref%get_started.cpp
    %atomic_three_get_started.cpp
    %Use Atomic Function%Constructor
%$$.

$end
-------------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/three_ctor.hpp
Constructors for atomic_three class.
*/

/*!
Base class for atomic_atomic functions.

\tparam Base
This class is used for defining an AD<Base> atomic operation y = g(x).

\par
make sure user does not invoke the default constructor
*/
template <class Base>
atomic_three<Base>::atomic_three(void)
{   CPPAD_ASSERT_KNOWN(false,
        "Attempt to use the atomic_three default constructor"
    );
}
/*!
Constructor

\param name
name used for error reporting
*/
// BEGIN_PROTOTYPE
template <class Base>
atomic_three<Base>::atomic_three(const std::string& name )
// END_PROTOTYPE
{   CPPAD_ASSERT_KNOWN(
        ! thread_alloc::in_parallel() ,
        "atomic_three: constructor cannot be called in parallel mode."
    );
    //
    // atomic_index
    bool        set_null  = false;
    size_t      index     = 0;
    size_t      type      = 3;
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
