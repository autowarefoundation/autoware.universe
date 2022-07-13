# ifndef CPPAD_CORE_DISCRETE_DISCRETE_HPP
# define CPPAD_CORE_DISCRETE_DISCRETE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <vector>
# include <cppad/core/cppad_assert.hpp>

// needed before one can use CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL
# include <cppad/utility/thread_alloc.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*
 ------------------------------------------------------------------------------
$begin discrete_create$$
$spell
$$
$section Create a Discrete AD Function$$

$head Syntax$$
$codei%CPPAD_DISCRETE_FUNCTION(%Base%, %name%)
%$$
$icode%name(%ax%, %ay%)
%$$

$head Base$$
is the base type for the discrete function.

$head name$$
is the name of the user defined function that corresponding to this operation.

$head ax$$
Is a $codei%AD<%Base%>%$$ corresponding to the argument for the function.

$head ay$$
Is a $codei%AD<%Base%>%$$ corresponding to the result for the function.

$head fun$$
The local object $code fun$$ is a member of the $code discrete$$ class.

$head Source Code$$
$srccode%hpp% */
# define CPPAD_DISCRETE_FUNCTION(Base, name)            \
inline CppAD::AD<Base> name (const CppAD::AD<Base>& ax) \
{    static CppAD::discrete<Base> fun(#name, name);     \
     return fun.ad(ax);                                 \
}
# define CppADCreateDiscrete CPPAD_DISCRETE_FUNCTION
/* %$$
$end
-----------------------------------------------------------------------------
$begin discrete_class$$

$section Declare discrete Class and Member Data$$

$head parallel_ad$$
is a friend of this class so it can call List to initialize
its static data.

$head F$$
is the type for the user routine that computes $icode Base$$ function values.

$head name_$$
name of this user defined discrete function.

$head f_$$
user routine that computes $icode Base$$ function values.

$head index_$$
index of this object in $cref discrete_list$$ for this $icode Base$$.

$head Source Code$$
$srccode%hpp% */
template <class Base>
class discrete {
private:
    template <class Type> friend void parallel_ad(void);
    typedef Base (*F) (const Base& x);
    const std::string    name_;
    const F              f_;
    const size_t         index_;
/* %$$
$end
------------------------------------------------------------------------------
$begin discrete_list$$
$spell
    alloc
    std
    CppAD
$$
$section List of all objects in the discrete class$$

$head Syntax$$
$icode%list% = discrete<%Base%>::List()%$$

$head Base$$
Is the $cref/Base/discrete_create/Base/$$
type for this list of discrete functions.

$head list$$
is a reference to the list of all the
$code discrete$$ object currently defined.

$subhead std::vector$$
We use $code std::vector$$ instead of $code CppAD::vector$$
so it does not appear that there is a $cref memory_leak$$
this list is not destroyed before
$cref/thread_alloc::free_all/ta_free_all/$$ is called by testing routines.

$head Source Code$$
$srccode%hpp% */
private:
    static std::vector<discrete *>& List(void)
    {   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;
        static std::vector<discrete *> list;
        return list;
    }
/* %$$
$end
 ------------------------------------------------------------------------------
$begin discrete_list_size$$
$spell
$$
$section Size of the Discrete Function List$$

$head Syntax$$
$icode%size% = discrete<%Base%>::list_size()%$$

$head Base$$
Is the $cref/Base/discrete_create/Base/$$
type for this list of discrete functions.

$head size$$
is the number of discrete functions for this $icode Base$$ type.

$head Source Code$$
$srccode%hpp% */
public:
    static size_t list_size(void)
    {   return List().size(); }
/* %$$
$end
 ------------------------------------------------------------------------------
$begin discrete_ctor$$
$spell
$$
$section Constructor Called by each Use of CPPAD_DISCRETE_FUNCTION$$

$head Syntax$$
$codei%discrete<%Base%> %fun%(%name%, %f%)%$$

$head name$$
is the name of this function.

$head f$$
user routine that implements this function for Base class.

$head fun$$
is the $code discrete$$ object created by this call to the constructor.

$subhead name_$$
is set equal to $icode name$$.

$subhead f_$$
is set equal to $icode f$$.

$subhead index_$$
This object is put at the end of $cref discrete_list$$ and $code index_$$
is set to the index of this object in the discrete list.

$head Parallel$$
This constructor cannot be used in parallel mode because it changes
the static object returned by $cref discrete_list$$.

$end
*/
public:
    discrete(const char* name, F f) :
    name_(name), f_(f) , index_( List().size() )
    {   std::string msg = "discrete: first call to the discrete function ";
        msg  += name;
        msg  += " is in parallel mode.";
        CPPAD_ASSERT_KNOWN(
            ! thread_alloc::in_parallel() ,
            msg.c_str()
        );
        List().push_back(this);
    }
/*
 ------------------------------------------------------------------------------
$begin discrete_ad$$
$spell
$$
$section Implement AD Version of a Discrete Function$$

$head Syntax$$
$icode%ay% = %fun%.ad(%ax)%$$

$head ax$$
is the argument for the AD version of this function.

$head ay$$
is the return value for the AD version of this function.

$head Prototype$$
$srccode%hpp% */
    AD<Base> ad(const AD<Base> &ax) const
/* %$$
$end
*/
    {
        CPPAD_ASSERT_KNOWN(
            size_t( std::numeric_limits<addr_t>::max() ) >= index_,
            "discrete: cppad_tape_addr_type maximum not large enough"
        );
        //
        AD<Base> ay;
        ay.value_ = f_(ax.value_);
        //
        // check if there is a recording in progress
        local::ADTape<Base>* tape = AD<Base>::tape_ptr();
        if( tape == nullptr )
            return ay;
        //
        // check if argument is a constant parameter
        if( ax.tape_id_ != tape->id_ )
            return ay;
        //
        if( ax.ad_type_ == dynamic_enum )
        {
            // tape dynamic paramter operation
            ay.taddr_   = tape->Rec_.put_dyn_par(
                ay.value_, local::dis_dyn, addr_t(index_), ax.taddr_
            );
            ay.tape_id_  = ax.tape_id_;
            ay.ad_type_  = dynamic_enum;

            // make result a dynamic parameter
            ay.tape_id_    = tape->id_;
            ay.ad_type_    = dynamic_enum;

            CPPAD_ASSERT_UNKNOWN( Dynamic(ay) );
        }
        else if( ax.ad_type_ == variable_enum )
        {
            CPPAD_ASSERT_UNKNOWN( local::NumRes(local::DisOp) == 1 );
            CPPAD_ASSERT_UNKNOWN( local::NumArg(local::DisOp) == 2 );

            // put operand addresses in the tape
            CPPAD_ASSERT_KNOWN(
                size_t( std::numeric_limits<addr_t>::max() ) >= index_,
                "discrete: cppad_tape_addr_type maximum not large enough"
            );
            tape->Rec_.PutArg(addr_t(index_), ax.taddr_);
            // put operator in the tape
            ay.taddr_ = tape->Rec_.PutOp(local::DisOp);
            // make result a variable
            ay.tape_id_    = tape->id_;
            ay.ad_type_    = variable_enum;

            CPPAD_ASSERT_UNKNOWN( Variable(ay) );
        }
        else
        {   // other types not yet being used and should have this tape id
            CPPAD_ASSERT_UNKNOWN(false);
        }
        return ay;
    }
/*
------------------------------------------------------------------------------
$begin discrete_name$$

$section Name Corresponding to a discrete Function$$

$head Syntax$$
$codei%discrete<%Base%>::name(%index%)%$$

$head Base$$
Is the $cref/Base/discrete_create/Base/$$
type for this list of discrete functions.

$head index$$
Is the index, in the list, for this discrete function.

$head Source Code$$
$srccode%hpp% */
    static const char* name(size_t index)
    {   return List()[index]->name_.c_str(); }
/* %$$
$end
------------------------------------------------------------------------------
$begin discrete_eval$$
$spell
    eval
$$
$section Link From Forward Mode Sweep to Users Routine$$

$head Syntax$$
$icode%y% = discrete<%Base%>::eval(%index%, %x%)%$$

$head Base$$
Is the $cref/Base/discrete_create/Base/$$
type for this list of discrete functions.

$head index$$
index for this function in $cref discrete_list$$.

$head x$$
argument at which to evaluate $icode Base$$ version of this function.

$head y$$
result for the $icode Base$$ version of this function.

$head Source Code$$
$srccode%hpp% */
    static Base eval(size_t index, const Base& x)
    {   CPPAD_ASSERT_UNKNOWN(index < List().size() );
        return List()[index]->f_(x);
    }
/* %$$
$end
*/
};

} // END_CPPAD_NAMESPACE
# endif
