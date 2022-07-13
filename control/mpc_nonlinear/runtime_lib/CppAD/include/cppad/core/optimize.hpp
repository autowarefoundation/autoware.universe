# ifndef CPPAD_CORE_OPTIMIZE_HPP
# define CPPAD_CORE_OPTIMIZE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# define CPPAD_CORE_OPTIMIZE_PRINT_RESULT 0

/*
$begin optimize$$
$spell
    enum
    jac
    bool
    Taylor
    CppAD
    cppad
    std
    const
    onetape
    op
    optimizer
$$

$section Optimize an ADFun Object Tape$$


$head Syntax$$
$icode%f%.optimize()
%$$
$icode%f%.optimize(%options%)
%$$
$icode%flag% = %f%.exceed_collision_limit()
%$$

$head Purpose$$
The operation sequence corresponding to an $cref ADFun$$ object can
be very large and involve many operations; see the
size functions in $cref fun_property$$.
The $icode%f%.optimize%$$ procedure reduces the number of operations,
and thereby the time and the memory, required to
compute function and derivative values.

$head f$$
The object $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$

$head options$$
This argument has prototype
$codei%
    const std::string& %options%
%$$
The default for $icode options$$ is the empty string.
If it is present, it must consist of one or more of the options below
separated by a single space character.

$subhead no_conditional_skip$$
The $code optimize$$ function can create conditional skip operators
to improve the speed of conditional expressions; see
$cref/optimize/CondExp/Optimize/$$.
If the sub-string $code no_conditional_skip$$ appears in $icode options$$,
conditional skip operations are not be generated.
This may make the optimize routine use significantly less memory
and take less time to optimize $icode f$$.
If conditional skip operations are generated,
it may save a significant amount of time when
using $icode f$$ for $cref forward$$ or $cref reverse$$ mode calculations;
see $cref number_skip$$.

$subhead no_compare_op$$
If the sub-string $code no_compare_op$$ appears in $icode options$$,
comparison operators will be removed from the optimized function.
These operators are necessary for the
$cref compare_change$$ functions to be meaningful.
On the other hand, they are not necessary, and take extra time,
when the compare_change functions are not used.

$subhead no_print_for_op$$
If the sub-string $code no_compare_op$$ appears in $icode options$$,
$cref PrintFor$$ operations will be removed form the optimized function.
These operators are useful for reporting problems evaluating derivatives
at independent variable values different from those used to record a function.

$subhead no_cumulative_sum_op$$
If this sub-string appears,
no cumulative sum operations will be generated during the optimization; see
$cref optimize_cumulative_sum.cpp$$.

$subhead collision_limit=value$$
If this substring appears,
where $icode value$$ is a sequence of decimal digits,
the optimizer's hash code collision limit will be set to $icode value$$.
When the collision limit is reached, the expressions with that hash code
are removed and a new lists of expressions with that has code is started.
The larger $icode value$$, the more identical expressions the optimizer
can recognize, but the slower the optimizer may run.
The default for $icode value$$ is $code 10$$.

$head Re-Optimize$$
Before 2019-06-28, optimizing twice was not supported and would fail
if cumulative sum operators were present after the first optimization.
This is now supported but it is not expected to have much benefit.
If you find a case where it does have a benefit, please inform the CppAD
developers of this.

$head Efficiency$$
If a $cref/zero order forward/forward_zero/$$ calculation is done during
the construction of $icode f$$, it will require more memory
and time than required after the optimization procedure.
In addition, it will need to be redone.
For this reason, it is more efficient to use
$codei%
    ADFun<%Base%> %f%;
    %f%.Dependent(%x%, %y%);
    %f%.optimize();
%$$
instead of
$codei%
    ADFun<%Base%> %f%(%x%, %y%)
    %f%.optimize();
%$$
See the discussion about
$cref/sequence constructors/FunConstruct/Sequence Constructor/$$.

$head Taylor Coefficients$$
Any Taylor coefficients in the function object are lost; i.e.,
$cref/f.size_order()/size_order/$$ after the optimization is zero.
(See the discussion about efficiency above.)

$head Speed Testing$$
You can run the CppAD $cref/speed/speed_main/$$ tests and see
the corresponding changes in number of variables and execution time.
Note that there is an interaction between using
$cref/optimize/speed_main/Global Options/optimize/$$ and
$cref/onetape/speed_main/Global Options/onetape/$$.
If $icode onetape$$ is true and $icode optimize$$ is true,
the optimized tape will be reused many times.
If $icode onetape$$ is false and $icode optimize$$ is true,
the tape will be re-optimized for each test.

$head Atomic Functions$$
There are some subtitle issue with optimized $cref atomic$$ functions
$latex v = g(u)$$:

$subhead rev_sparse_jac$$
The $cref atomic_two_rev_sparse_jac$$ function is be used to determine
which components of $icode u$$ affect the dependent variables of $icode f$$.
For each atomic operation, the current
$cref/atomic_sparsity/atomic_two_option/atomic_sparsity/$$ setting is used
to determine if $code pack_sparsity_enum$$, $code bool_sparsity_enum$$,
or $code set_sparsity_enum$$ is used to determine dependency relations
between argument and result variables.

$subhead nan$$
If $icode%u%[%i%]%$$ does not affect the value of
the dependent variables for $icode f$$,
the value of $icode%u%[%i%]%$$ is set to $cref nan$$.

$head Checking Optimization$$
If $cref/NDEBUG/Faq/Speed/NDEBUG/$$ is not defined,
and $cref/f.size_order()/size_order/$$ is greater than zero,
a $cref forward_zero$$ calculation is done using the optimized version
of $icode f$$ and the results are checked to see that they are
the same as before.
If they are not the same, the
$cref ErrorHandler$$ is called with a known error message
related to $icode%f%.optimize()%$$.

$head exceed_collision_limit$$
If the return value $icode flag$$ is true (false),
the previous call to $icode%f%.optimize%$$ exceed the
$cref/collision_limit/optimize/options/collision_limit=value/$$.

$head Examples$$
$comment childtable without Example instead of Contents for header$$
$children%
    example/optimize/optimize_twice.cpp
    %example/optimize/forward_active.cpp
    %example/optimize/reverse_active.cpp
    %example/optimize/compare_op.cpp
    %example/optimize/print_for.cpp
    %example/optimize/conditional_skip.cpp
    %example/optimize/nest_conditional.cpp
    %example/optimize/cumulative_sum.cpp
%$$
$table
$rref optimize_twice.cpp$$
$rref optimize_forward_active.cpp$$
$rref optimize_reverse_active.cpp$$
$rref optimize_compare_op.cpp$$
$rref optimize_print_for.cpp$$
$rref optimize_conditional_skip.cpp$$
$rref optimize_nest_conditional.cpp$$
$rref optimize_cumulative_sum.cpp$$
$tend

$end
-----------------------------------------------------------------------------
*/
# include <cppad/local/optimize/optimize_run.hpp>
/*!
\file optimize.hpp
Optimize a player object operation sequence
*/
namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
Optimize a player object operation sequence

The operation sequence for this object is replaced by one with fewer operations
but the same funcition and derivative values.

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type
 Base.

\param options
\li
If the sub-string "no_conditional_skip" appears,
conditional skip operations will not be generated.
This may make the optimize routine use significantly less memory
and take significantly less time.
\li
If the sub-string "no_compare_op" appears,
then comparison operators will be removed from the optimized tape.
These operators are necessary for the compare_change function to be
be meaningful in the resulting recording.
On the other hand, they are not necessary and take extra time
when compare_change is not used.
*/
template <class Base, class RecBase>
void ADFun<Base,RecBase>::optimize(const std::string& options)
{
# if CPPAD_CORE_OPTIMIZE_PRINT_RESULT
    // size of operation sequence before optimizatiton
    size_t size_op_before = size_op();
# endif

    // place to store the optimized version of the recording
    local::recorder<Base> rec;

    // number of independent variables
    size_t n = ind_taddr_.size();

# ifndef NDEBUG
    size_t i, j, m = dep_taddr_.size();
    CppAD::vector<Base> x(n), y(m), check(m);
    Base max_taylor(0);
    bool check_zero_order = num_order_taylor_ > 0;
    if( check_zero_order )
    {   // zero order coefficients for independent vars
        for(j = 0; j < n; j++)
        {   CPPAD_ASSERT_UNKNOWN( play_.GetOp(j+1) == local::InvOp );
            CPPAD_ASSERT_UNKNOWN( ind_taddr_[j]    == j+1   );
            x[j] = taylor_[ ind_taddr_[j] * cap_order_taylor_ + 0];
        }
        // zero order coefficients for dependent vars
        for(i = 0; i < m; i++)
        {   CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_  );
            y[i] = taylor_[ dep_taddr_[i] * cap_order_taylor_ + 0];
        }
        // maximum zero order coefficient not counting BeginOp at beginning
        // (which is correpsonds to uninitialized memory).
        for(i = 1; i < num_var_tape_; i++)
        {   if(  abs_geq(taylor_[i*cap_order_taylor_+0] , max_taylor) )
                max_taylor = taylor_[i*cap_order_taylor_+0];
        }
    }
# endif

    // create the optimized recording
    size_t exceed = false;
    switch( play_.address_type() )
    {
        case local::play::unsigned_short_enum:
        exceed = local::optimize::optimize_run<unsigned short>(
            options, n, dep_taddr_, &play_, &rec
        );
        break;

        case local::play::unsigned_int_enum:
        exceed = local::optimize::optimize_run<unsigned int>(
            options, n, dep_taddr_, &play_, &rec
        );
        break;

        case local::play::size_t_enum:
        exceed = local::optimize::optimize_run<size_t>(
            options, n, dep_taddr_, &play_, &rec
        );
        break;

        default:
        CPPAD_ASSERT_UNKNOWN(false);
    }
    exceed_collision_limit_ = exceed;

    // number of variables in the recording
    num_var_tape_  = rec.num_var_rec();

    // now replace the recording
    play_.get_recording(rec, n);

    // set flag so this function knows it has been optimized
    has_been_optimized_ = true;

    // free memory allocated for sparse Jacobian calculation
    // (the results are no longer valid)
    for_jac_sparse_pack_.resize(0, 0);
    for_jac_sparse_set_.resize(0,0);

    // free old Taylor coefficient memory
    taylor_.clear();
    num_order_taylor_     = 0;
    cap_order_taylor_     = 0;

    // resize and initilaize conditional skip vector
    // (must use player size because it now has the recoreder information)
    cskip_op_.resize( play_.num_op_rec() );

    // resize subgraph_info_
    subgraph_info_.resize(
        ind_taddr_.size(),    // n_ind
        dep_taddr_.size(),    // n_dep
        play_.num_op_rec(),   // n_op
        play_.num_var_rec()   // n_var
    );

# ifndef NDEBUG
    if( check_zero_order )
    {   std::stringstream s;
        //
        // zero order forward calculation using new operation sequence
        check = Forward(0, x, s);

        // check results
        Base eps99 = Base(99) * CppAD::numeric_limits<Base>::epsilon();
        for(i = 0; i < m; i++)
        if( ! abs_geq( eps99 * max_taylor , check[i] - y[i] ) )
        {   std::string msg = "Error during check of f.optimize().";
            msg += "\neps99 * max_taylor = " + to_string(eps99 * max_taylor);
            msg += "\ncheck[i] = " + to_string(check[i]);
            msg += "\ny[i]     = " + to_string(y[i]);
            CPPAD_ASSERT_KNOWN(
                abs_geq( eps99 * max_taylor , check[i] - y[i] ) ,
                msg.c_str()
            );
        }

        // Erase memory that this calculation was done so NDEBUG gives
        // same final state for this object (from users perspective)
        num_order_taylor_     = 0;
    }
# endif
# if CPPAD_CORE_OPTIMIZE_PRINT_RESULT
    // size of operation sequence after optimizatiton
    size_t size_op_after = size_op();
    std::cout << "optimize: size_op:  before = " <<
    size_op_before << ", after = " << size_op_after << "\n";
# endif
}

} // END_CPPAD_NAMESPACE

# undef CPPAD_CORE_OPTIMIZE_PRINT_RESULT
# endif
