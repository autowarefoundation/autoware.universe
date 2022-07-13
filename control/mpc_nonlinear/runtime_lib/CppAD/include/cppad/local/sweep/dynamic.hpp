# ifndef CPPAD_LOCAL_SWEEP_DYNAMIC_HPP
# define CPPAD_LOCAL_SWEEP_DYNAMIC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/play/atom_op_info.hpp>

// BEGIN_CPPAD_LOCAL_SWEEP_NAMESPACE
namespace CppAD { namespace local { namespace sweep {
/*!
\file sweep/dynamic.hpp
Under Consruction
*/

/*!
\def CPPAD_DYNAMIC_TRACE
This value is either zero or one.
Zero is the normal operational value.
If it is one, a trace for each dynamic parameter is compuation is printed.
Sometimes it is usefull to trace f.new_dynamic with the same
dynamic parameter values as during the recording
(to debug the recording process).
*/
# define CPPAD_DYNAMIC_TRACE 0

/*!
Compute dynamic parameters.

\tparam Base
The type of the parameters.

\tparam BaseVector
is a simple vector class with elements of type Base.

\param ind_dynamic
new value for the independent dynamic parameter vector.

\param all_par_vec
is the vector of all the parameters.
Ths constant parameters are inputs and the dynamic parameters are outputs.

\param dyn_par_is
is a vector with the same length as par_vec.
The i-th parameter is dynamic if and only if dyn_par_is[i] is true.

\param dyn_ind2par_ind
is a vector with length equal to the number of dynamic parameters.
The element dyn_ind2par_ind[j] is the index in all_par_vec corresponding
to the j-th dynamic parameter.
Note that if dyn_par_is[i] is false, the i-th parameter does not
appear in this vector.

\param dyn_par_op
is a vector with length equal to the number of dynamic parameters.
The element dyn_par_op_[j] is the operator for the j-th dynamic parameter.
Note that if dyn_par_is[i] is false, the i-th parameter does not
have a parameter in this list.

\param dyn_par_arg
is a vector containing the arguments for the dynamic parameters.
The first argument for the j-th dynamic parameter is dyn_par_arg[k]
where
\code
    k = NumArg( dyn_par_op[0] ) + ... + NumArg( dyn_par_op[j-1] )
\endcode
The arguments for each dynamic parameter have index value
lower than the index value for the parameter.

\param not_used_rec_base
Specifies RecBase for this call.
*/
template <class Base, class BaseVector, class RecBase>
void dynamic(
    pod_vector_maybe<Base>&       all_par_vec        ,
    const BaseVector&             ind_dynamic        ,
    const pod_vector<bool>&       dyn_par_is         ,
    const pod_vector<addr_t>&     dyn_ind2par_ind    ,
    const pod_vector<opcode_t>&   dyn_par_op         ,
    const pod_vector<addr_t>&     dyn_par_arg        ,
    const RecBase&                not_used_rec_base  )
{
    // number of dynamic parameters
    size_t num_dynamic_par = dyn_ind2par_ind.size();

    // vectors used in call to atomic fuctions
    vector<ad_type_enum> type_x;
    vector<Base>         taylor_x, taylor_y;
# ifndef NDEBUG
    for(size_t j = 0; j < ind_dynamic.size(); ++j)
        CPPAD_ASSERT_UNKNOWN(
            dyn_par_is[j+1] && op_code_dyn( dyn_par_op[j] ) == ind_dyn
    );
# endif
# if CPPAD_DYNAMIC_TRACE
    const char* cond_exp_name[] = {
        "CondExpLt",
        "CondExpLe",
        "CondExpEq",
        "CondExpGe",
        "CondExpGt",
        "CondExpNe"
    };
    std::cout
    << std::endl
    << std::setw(10) << std::left << "index"
    << std::setw(10) << std::left << "old"
    << std::setw(10) << std::left << "new"
    << std::setw(11) << std::left << "op"
    << std::setw(26) << std::right << "dynamic i=, constant v="
    << std::endl;
# endif
    // used to hold the first two parameter arguments
    const Base* par[2];
    for(size_t j = 0; j < 2; ++j)
        par[j] = nullptr;
    //
    // Initialize index in dyn_par_arg
    size_t i_arg = 0;
    //
    // Loop throubh the dynamic parameters
    size_t i_dyn = 0;
    while(i_dyn < num_dynamic_par)
    {   // number of dynamic parameters created by this operator
        size_t n_dyn = 1;
        //
        // parameter index for this dynamic parameter
        size_t i_par = size_t( dyn_ind2par_ind[i_dyn] );
        //
# if CPPAD_DYNAMIC_TRACE
        Base old_value = all_par_vec[i_par];
# endif
        //
        // operator for this dynamic parameter
        op_code_dyn op = op_code_dyn( dyn_par_op[i_dyn] );
        //
        // number of arguments for this operator
        size_t n_arg       = num_arg_dyn(op);
        //
        // for unary or binary operators
        bool unary_or_binary = true;
        unary_or_binary &= op != atom_dyn;
        unary_or_binary &= op != cond_exp_dyn;
        unary_or_binary &= op != dis_dyn;
        unary_or_binary &= op != ind_dyn;
        unary_or_binary &= op != result_dyn;
        if( unary_or_binary )
        {   CPPAD_ASSERT_UNKNOWN( n_arg == 1 || n_arg == 2 );
           for(size_t j = 0; j < n_arg; ++j)
                par[j] = & all_par_vec[ dyn_par_arg[i_arg + j] ];
        }
        //
        switch(op)
        {
            // ---------------------------------------------------------------
            // standard_math_98
            // acos
            case acos_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = acos( *par[0] );
            break;

            // asin
            case asin_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = asin( *par[0] );
            break;

            // atan
            case atan_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = atan( *par[0] );
            break;

            // cos
            case cos_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = cos( *par[0] );
            break;

            // cosh
            case cosh_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = cosh( *par[0] );
            break;

            // ind
            case ind_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 0 );
            CPPAD_ASSERT_UNKNOWN( i_par == i_dyn + 1 );
            all_par_vec[i_par] = ind_dynamic[i_dyn];
            break;

            // exp
            case exp_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = exp( *par[0] );
            break;

            // fabs
            case fabs_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = fabs( *par[0] );
            break;

            // log
            case log_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = log( *par[0] );
            break;

            // sin
            case sin_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = sin( *par[0] );
            break;

            // sinh
            case sinh_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = sinh( *par[0] );
            break;

            // sqrt
            case sqrt_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = sqrt( *par[0] );
            break;

            // tan
            case tan_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = tan( *par[0] );
            break;

            // tanh
            case tanh_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = tanh( *par[0] );
            break;

            // ---------------------------------------------------------------
            // asinh
            case asinh_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = asinh( *par[0] );
            break;

            // acosh
            case acosh_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = acosh( *par[0] );
            break;

            // atanh
            case atanh_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = atanh( *par[0] );
            break;

            // expm1
            case expm1_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = expm1( *par[0] );
            break;

            // erf
            case erf_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = erf( *par[0] );
            break;

            // erfc
            case erfc_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = erfc( *par[0] );
            break;

            // log1p
            case log1p_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = log1p( *par[0] );
            break;
            // ---------------------------------------------------------------
            // abs
            case abs_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = fabs( *par[0] );
            break;

            // add
            case add_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 2 );
            all_par_vec[i_par] = *par[0] + *par[1];
            break;

            // div
            case div_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 2 );
            all_par_vec[i_par] = *par[0] / *par[1];
            break;

            // mul
            case mul_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 2 );
            all_par_vec[i_par] = *par[0] * *par[1];
            break;

            // neg
            case neg_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = - *par[0];
            break;

            // pow
            case pow_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 2 );
            all_par_vec[i_par] = pow( *par[0], *par[1] );
            break;

            // sign
            case sign_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 1 );
            all_par_vec[i_par] = sign( *par[0] );
            break;

            // sub
            case sub_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 2 );
            all_par_vec[i_par] = *par[0] - *par[1];
            break;

            // zmul
            case zmul_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 2 );
            all_par_vec[i_par] = azmul( *par[0],  *par[1] );
            break;

            // ---------------------------------------------------------------
            // discrete(index, argument)
            case dis_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 2 );
            all_par_vec[i_par] = discrete<Base>::eval(
                size_t(      dyn_par_arg[i_arg + 0] ) , // index
                all_par_vec[ dyn_par_arg[i_arg + 1] ]   // argument
            );
# if CPPAD_DYNAMIC_TRACE
            std::cout
            << std::setw(10) << std::left << i_par
            << std::setw(10) << std::left << old_value
            << std::setw(10) << std::left << all_par_vec[i_par]
            << "="
            << std::setw(10) << std::right << op_name_dyn(op)
            << "("
            << std::setw(12) << std::right <<
                discrete<Base>::name( size_t( dyn_par_arg[i_arg + 0] ) );
            if( dyn_par_is[ dyn_par_arg[i_arg + 1] ] )
            {   std::cout << ", i=" << std::setw(10) << std::right
                << dyn_par_arg[i_arg + 1];
            }
            else
            {   std::cout << ", v=" << std::setw(10) << std::right
                << all_par_vec[ dyn_par_arg[i_arg + 1] ];
            }
            std::cout << ")" << std::endl;
# endif
            break;

            // ---------------------------------------------------------------
            // cond_exp(cop, left, right, if_true, if_false)
            // (not yet implemented)
            case cond_exp_dyn:
            CPPAD_ASSERT_UNKNOWN( n_arg == 5 );
            all_par_vec[i_par] = CondExpOp(
                CompareOp(   dyn_par_arg[i_arg + 0] ) , // cop
                all_par_vec[ dyn_par_arg[i_arg + 1] ] , // left
                all_par_vec[ dyn_par_arg[i_arg + 2] ] , // right
                all_par_vec[ dyn_par_arg[i_arg + 3] ] , // if_true
                all_par_vec[ dyn_par_arg[i_arg + 4] ]   // if_false
            );
# if CPPAD_DYNAMIC_TRACE
            std::cout
            << std::setw(10) << std::left << i_par
            << std::setw(10) << std::left << old_value
            << std::setw(10) << std::left << all_par_vec[i_par]
            << "="
            << std::setw(10) << std::right
            << cond_exp_name[ dyn_par_arg[i_arg + 0] ]
            << "(";
            for(size_t i = 1; i < 5; ++i)
            {   if( dyn_par_is[ dyn_par_arg[i_arg + i] ] )
                {   std::cout << "i=" << std::setw(10) << std::right
                    << dyn_par_arg[i_arg + i];
                }
                else
                {   std::cout << "v=" << std::setw(10) << std::right
                    << all_par_vec[ dyn_par_arg[i_arg + i] ];
                }
                if( i < 4 )
                    std::cout << ",";
            }
            std::cout << ")" << std::endl;
# endif
            break;
            // ---------------------------------------------------------------
            // atomic function results
            case result_dyn:
            break;

            // atomic function call
            case atom_dyn:
            {   size_t atom_index = size_t( dyn_par_arg[i_arg + 0] );
                size_t n          = size_t( dyn_par_arg[i_arg + 1] );
                size_t m          = size_t( dyn_par_arg[i_arg + 2] );
                n_dyn             = size_t( dyn_par_arg[i_arg + 3] );
                n_arg             = 5 + n + m;
                CPPAD_ASSERT_UNKNOWN(
                    size_t( dyn_par_arg[i_arg + 4 + n + m] ) == n_arg
                );
                //
                size_t need_y    = size_t(dynamic_enum);
                size_t order_low = 0;
                size_t order_up  = 0;
                size_t atom_old  = 0; // not used
                type_x.resize(n);
                taylor_x.resize(n);
                taylor_y.resize(m);
                for(size_t j = 0; j < n; ++j)
                {   addr_t arg_j = dyn_par_arg[i_arg + 4 + j];
                    taylor_x[j]   = all_par_vec[ arg_j ];
                    if( arg_j == 0 )
                        type_x[j] = variable_enum;
                    else if ( dyn_par_is[arg_j] )
                        type_x[j] = dynamic_enum;
                    else
                        type_x[j] = constant_enum;
                }
                call_atomic_forward<Base, RecBase>(
                    taylor_x,
                    type_x,
                    need_y,
                    order_low,
                    order_up,
                    atom_index,
                    atom_old,
                    taylor_x,
                    taylor_y
                );
# if CPPAD_DYNAMIC_TRACE
                // get the name of this atomic function
                bool         set_null = false;
                size_t       type     = 0;          // set to avoid warning
                std::string name;
                void*        v_ptr    = nullptr; // set to avoid warning
                atomic_index<RecBase>(
                    set_null, atom_index, type, &name, v_ptr
                );
                std::cout << "atom_dyn " << name << " arguments\n";
                for(size_t j = 0; j < n; ++j)
                {   std::cout << "index = " << j
                    << ", value = " << taylor_x[j] << std::endl;
                }
                std::cout << "atom_dyn " << name << " results\n";
# endif
# ifndef NDEBUG
                size_t count_dyn = 0;
# endif
                for(size_t i = 0; i < m; ++i)
                {   i_par = size_t( dyn_par_arg[i_arg + 4 + n + i] );
                    if( dyn_par_is[i_par] )
                    {   CPPAD_ASSERT_UNKNOWN( i_par != 0 );
                        all_par_vec[i_par] = taylor_y[i];
# ifndef NDEBUG
                        ++count_dyn;
# endif
# if CPPAD_DYNAMIC_TRACE
                        std::cout
                        << std::setw(10) << std::left << i_par
                        << std::setw(10) << std::left << old_value
                        << std::setw(10) << std::left << all_par_vec[i_par]
                        << "= " << name << "_" << i << std::endl;
# endif
                    }
                }
                CPPAD_ASSERT_UNKNOWN( count_dyn == n_dyn );
# if CPPAD_DYNAMIC_TRACE
                std::cout << "end atomic dynamic parameter results\n";
# endif
            }
            break;

            // ---------------------------------------------------------------
            default:
            std::cerr << "op_code_dyn = " << op_name_dyn(op) << std::endl;
            CPPAD_ASSERT_UNKNOWN(false);
            break;
        }
# if CPPAD_DYNAMIC_TRACE
        if(
            (op != cond_exp_dyn) &
            (op != dis_dyn )     &
            (op != atom_dyn )    &
            (op != result_dyn )  )
        {
            std::cout
            << std::setw(10) << std::left << i_par
            << std::setw(10) << std::left << old_value
            << std::setw(10) << std::left << all_par_vec[i_par]
            << "="
            << std::setw(10) << std::right << op_name_dyn(op)
            << "(";
            if( 0 < n_arg )
            {   if( dyn_par_is[ dyn_par_arg[i_arg + 0] ] )
                {   std::cout << "i=" << std::setw(10) << std::right
                    << dyn_par_arg[i_arg + 0];
                }
                else
                {   std::cout << "v=" << std::setw(10) << std::right
                    << all_par_vec[ dyn_par_arg[i_arg + 0] ];
                }
            }
            if( 1 < n_arg )
            {   if( dyn_par_is[ dyn_par_arg[i_arg + 1] ] )
                {   std::cout << ", i=" << std::setw(10) << std::right
                    << dyn_par_arg[i_arg + 1];
                }
                else
                {   std::cout << ", v=" << std::setw(10) << std::right
                    << all_par_vec[ dyn_par_arg[i_arg + 1] ];
                }
            }
            std::cout << ")" << std::endl;
        }
# endif
        i_arg += n_arg;
        i_dyn += n_dyn;
    }
    CPPAD_ASSERT_UNKNOWN( i_arg == dyn_par_arg.size() )
    return;
}

// preprocessor symbols that are local to this file
# undef CPPAD_DYNAMIC_TRACE

} } } // END_CPPAD_LOCAL_SWEEP_NAMESPACE

# endif
