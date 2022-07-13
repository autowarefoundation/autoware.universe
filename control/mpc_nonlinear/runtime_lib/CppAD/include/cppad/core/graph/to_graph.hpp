# ifndef CPPAD_CORE_GRAPH_TO_GRAPH_HPP
# define CPPAD_CORE_GRAPH_TO_GRAPH_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/core/ad_fun.hpp>
# include <cppad/local/op_code_dyn.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>

/*
------------------------------------------------------------------------------
$begin to_graph$$
$spell
    Json
    cpp
    ind
    vec
    arg
    obj
    op_enum
$$

$section Create a C++ AD Graph Corresponding to an ADFun Object$$

$head Syntax$$
$codei%
    cpp_graph %graph_obj%
    ADFun<%Base%> %fun%
    %fun%.to_graph(%graph_obj%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head Base$$
is the type corresponding to this $cref/ADFun/adfun/$$ object;
i.e., its calculations are done using the type $icode Base$$.

$head RecBase$$
in the prototype above, $icode RecBase$$ is the same type as $icode Base$$.

$head graph_obj$$
This is a $code cpp_graph$$ object.
The input value of the object does not matter.
Upon return it is a $cref cpp_ad_graph$$ representation of this function.

$head Restrictions$$
The $code to_graph$$ routine is not yet implement for some
possible $cref ADFun$$ operators; see
$cref/missing operators/graph_op_enum/Missing Operators/$$.

$head Examples$$
See $cref/graph_op_enum examples/graph_op_enum/Examples/$$.

$end
*/
// BEGIN_PROTOTYPE
template <class Base, class RecBase>
void CppAD::ADFun<Base,RecBase>::to_graph(
        CppAD::cpp_graph& graph_obj )
// END_PROTOTYPE
{   using local::pod_vector;
    using local::opcode_t;
    using namespace CppAD::graph;
    //
# ifndef NDEBUG
# endif
    graph_obj.initialize();
    //
    // --------------------------------------------------------------------
    // some constants
    // --------------------------------------------------------------------
    //
    // output: function_name
    graph_obj.function_name_set(function_name_);
    //
    // dynamic parameter information
    const pod_vector<opcode_t>& dyn_par_op ( play_.dyn_par_op()  );
    const pod_vector<addr_t>&   dyn_par_arg( play_.dyn_par_arg() );
    const pod_vector<addr_t>&   dyn_ind2par_ind ( play_.dyn_ind2par_ind() );
    const pod_vector<bool>&     dyn_par_is( play_.dyn_par_is() );
    //
    // number of dynamic parameters
    const size_t n_dynamic     = dyn_ind2par_ind.size();
    //
    // output: n_dynamic_ind
    size_t n_dynamic_ind = play_.num_dynamic_ind();
    graph_obj.n_dynamic_ind_set(n_dynamic_ind);
    //
    // number of parameters
    const size_t n_parameter = play_.num_par_rec();
    //
    // number of constant parameters
# ifndef NDEBUG
    const size_t n_constant = n_parameter - n_dynamic - 1;
# endif
    //
    // output: n_variable_ind
    size_t n_variable_ind = ind_taddr_.size();
    graph_obj.n_variable_ind_set(n_variable_ind);
    //
    // value of parameters
    const Base* parameter = play_.GetPar();
    //
    // number of variables
    const size_t n_variable = play_.num_var_rec();
    //
    // some checks
    CPPAD_ASSERT_UNKNOWN( n_dynamic_ind <= n_dynamic );
    CPPAD_ASSERT_UNKNOWN( dyn_par_is.size() == n_parameter );
    CPPAD_ASSERT_UNKNOWN( n_parameter > 0 );
    CPPAD_ASSERT_UNKNOWN( isnan( parameter[0] ) );
    CPPAD_ASSERT_UNKNOWN( ! dyn_par_is[0] );
    // --------------------------------------------------------------------
    // par2node
    pod_vector<size_t> par2node(n_parameter);
    par2node[0] = 0; // invalid value
    for(size_t i = 1; i <= n_dynamic_ind; ++i)
        par2node[i] = i; // independent dynamic parameters
    for(size_t i = n_dynamic_ind + 1; i < n_parameter; ++i)
        par2node[i] = 0; // will be set later
    // ----------------------------------------------------------------------
    //
    // initialize index of previous node in the graph
    size_t previous_node = 0;
    //
    // n_dynamic_ind
    previous_node += n_dynamic_ind;
    //
    // n_variable_ind
    previous_node += n_variable_ind;
    // --------------------------------------------------------------------
    //
    // output: constant_vec
    // constant_vec and par2node for constants
    for(size_t i = 1; i < n_parameter; ++i)
    {   if( ! dyn_par_is[i] )
        {   // this is a constant node
            graph_obj.constant_vec_push_back( parameter[i] );
            par2node[i] = ++previous_node;
        }
    }
    CPPAD_ASSERT_UNKNOWN( n_constant == graph_obj.constant_vec_size() );
    // ----------------------------------------------------------------------
    //  output: initialize atomic_name_vec, operator_vec, operator_arg
    // temporary used for elements of operator_vec
    //
    // Json operators are dynamic operators plus variables operators.
    // Skip BeginOp, EndOp, and independent variables.
    //
    // dynamic parameter operations and par2node
    // for dynamic parameters that are not constants or independent
    CPPAD_ASSERT_UNKNOWN( num_arg_dyn(local::ind_dyn) == 0 );
    CPPAD_ASSERT_UNKNOWN( num_arg_dyn(local::atom_dyn) == 0 );
    size_t i_arg = 0;
    pod_vector<size_t> node_arg;

    for(size_t i_dyn = n_dynamic_ind; i_dyn < n_dynamic; ++i_dyn)
    {   // operator for this dynamic parameter
        local::op_code_dyn dyn_op = local::op_code_dyn( dyn_par_op[i_dyn] );
        //
        // parameter index for this dynamic parameter
        size_t i_par = size_t( dyn_ind2par_ind[i_dyn] );
        CPPAD_ASSERT_UNKNOWN( par2node[i_par] == 0 );
        par2node[i_par] = ++previous_node;
        //
        // number of arguments for operators with exception of atom_dyn
        size_t n_arg = num_arg_dyn(dyn_op);
        if( n_arg > node_arg.size() )
            node_arg.resize(n_arg);
        //
        // parameter arguments in graph node space (except for atom_dyn)
        if( dyn_op != local::atom_dyn )
        {   size_t offset_par = num_non_par_arg_dyn(dyn_op);
            for(size_t i = offset_par; i < n_arg; ++i)
            {   node_arg[i] = par2node[ dyn_par_arg[i_arg + i] ];
                CPPAD_ASSERT_UNKNOWN( node_arg[i] > 0 );
            }
        }
        //
        // invalid value
        graph_op_enum graph_op = n_graph_op;
        switch(dyn_op)
        {
            // ---------------------------------------------------------------
            // unary operators

            case local::abs_dyn:
            case local::fabs_dyn:
            graph_op = abs_graph_op;
            break;

            case local::acosh_dyn:
            graph_op = acosh_graph_op;
            break;

            case local::asinh_dyn:
            graph_op = asinh_graph_op;
            break;

            case local::atanh_dyn:
            graph_op = atanh_graph_op;
            break;

            case local::erf_dyn:
            graph_op = erf_graph_op;
            break;

            case local::erfc_dyn:
            graph_op = erfc_graph_op;
            break;

            case local::expm1_dyn:
            graph_op = expm1_graph_op;
            break;

            case local::log1p_dyn:
            graph_op = log1p_graph_op;
            break;

            case local::acos_dyn:
            graph_op = acos_graph_op;
            break;

            case local::asin_dyn:
            graph_op = asin_graph_op;
            break;

            case local::atan_dyn:
            graph_op = atan_graph_op;
            break;

            case local::cosh_dyn:
            graph_op = cosh_graph_op;
            break;

            case local::cos_dyn:
            graph_op = cos_graph_op;
            break;

            case local::exp_dyn:
            graph_op = exp_graph_op;
            break;

            case local::log_dyn:
            graph_op = log_graph_op;
            break;

            case local::neg_dyn:
            graph_op = neg_graph_op;
            break;

            case local::sign_dyn:
            graph_op = sign_graph_op;
            break;

            case local::sinh_dyn:
            graph_op = sinh_graph_op;
            break;

            case local::sin_dyn:
            graph_op = sin_graph_op;
            break;

            case local::sqrt_dyn:
            graph_op = sqrt_graph_op;
            break;

            case local::tanh_dyn:
            graph_op = tanh_graph_op;
            break;

            case local::tan_dyn:
            graph_op = tan_graph_op;
            break;

            // ---------------------------------------------------------------
            // binary operators

            case local::add_dyn:
            graph_op = add_graph_op;
            break;

            case local::div_dyn:
            graph_op = div_graph_op;
            break;

            case local::mul_dyn:
            graph_op = mul_graph_op;
            break;

            case local::pow_dyn:
            graph_op = pow_graph_op;
            break;

            case local::sub_dyn:
            graph_op = sub_graph_op;
            break;

            case local::zmul_dyn:
            graph_op = azmul_graph_op;
            break;

            // ---------------------------------------------------------------
            // graph_op determined later for these cases
            case local::atom_dyn:
            case local::cond_exp_dyn:
            case local::dis_dyn:
            case local::result_dyn:
            break;

            // ---------------------------------------------------------------
            default:
            // This error should have been reported above
            CPPAD_ASSERT_UNKNOWN( false );
            break;
        }
        switch( dyn_op )
        {   // --------------------------------------------------------------
            case local::result_dyn:
            // setting par2node[i_dyn] above is all that is necessary
            CPPAD_ASSERT_UNKNOWN( n_arg == 0 );
            break;

            // --------------------------------------------------------------
            case local::dis_dyn:
            {
                // arg[0]: discrete function index
                size_t discrete_index = size_t( dyn_par_arg[i_arg + 0] );
                // get the name for this dicrete function
                std::string name = discrete<Base>::name( discrete_index );
                //
                // set graph index for this discrete function call
                size_t name_index = graph_obj.discrete_name_vec_find(name);
                if( name_index == graph_obj.discrete_name_vec_size() )
                    graph_obj.discrete_name_vec_push_back(name);
                //
                graph_op = discrete_graph_op;
                graph_obj.operator_vec_push_back( graph_op );
                graph_obj.operator_arg_push_back( name_index );
                graph_obj.operator_arg_push_back( node_arg[1] );
            }
            break;

            // --------------------------------------------------------------
            case local::atom_dyn:
            {
                // arg[0]: atomic function index
                size_t atom_index  = size_t( dyn_par_arg[i_arg + 0] );
                // arg[1]: number of arguments to function
                n_arg              = size_t( dyn_par_arg[i_arg + 1] );
                // arg[2]: number of results from function
                size_t n_result    = size_t( dyn_par_arg[i_arg + 2] );
                //
                // get the name for this atomic function
                std::string     name;
                {   bool        set_null = false;
                    size_t      type;
                    void*       ptr;
                    CppAD::local::atomic_index<RecBase>(
                        set_null, atom_index, type, &name, ptr
                    );
                }
                // set graph index for this atomic function call
                size_t name_index = graph_obj.atomic_name_vec_find(name);
                if( name_index == graph_obj.atomic_name_vec_size() )
                    graph_obj.atomic_name_vec_push_back(name);
                //
                // for atom_graph_op:
                // name_index, n_result, n_arg come before first_node
                graph_obj.operator_arg_push_back(name_index);
                graph_obj.operator_arg_push_back(n_result);
                graph_obj.operator_arg_push_back(n_arg);
                //
                graph_op = atom_graph_op;
                graph_obj.operator_vec_push_back( graph_op );
                //
                for(size_t j  = 0; j < n_arg; ++j)
                {   // arg[4 + j] is j-th argument to the function
                    size_t node_j = par2node[ dyn_par_arg[i_arg + 4 + j] ];
                    CPPAD_ASSERT_UNKNOWN( node_j < i_par );
                    graph_obj.operator_arg_push_back( node_j );
                }
            }
            break;

            // --------------------------------------------------------------
            case local::cond_exp_dyn:
            {
                CompareOp cop = CompareOp( dyn_par_arg[i_arg + 0] );
                size_t left     = node_arg[1];
                size_t right    = node_arg[2];
                size_t if_true  = node_arg[3];
                size_t if_false = node_arg[4];
                switch( cop )
                {   case CompareLt:
                    graph_op = cexp_lt_graph_op;
                    break;

                    case CompareLe:
                    graph_op = cexp_le_graph_op;
                    break;

                    case CompareEq:
                    graph_op = cexp_eq_graph_op;
                    break;

                    case CompareGe:
                    graph_op = cexp_lt_graph_op;
                    std::swap(if_true, if_false);
                    break;

                    case CompareGt:
                    graph_op = cexp_le_graph_op;
                    std::swap(if_true, if_false);
                    break;

                    case CompareNe:
                    graph_op = cexp_eq_graph_op;
                    std::swap(if_true, if_false);
                    break;

                    default:
                    CPPAD_ASSERT_UNKNOWN(false);
                    break;
                }
                graph_obj.operator_vec_push_back( graph_op );
                graph_obj.operator_arg_push_back( left );
                graph_obj.operator_arg_push_back( right );
                graph_obj.operator_arg_push_back( if_true );
                graph_obj.operator_arg_push_back( if_false );
            }
            break;

            // --------------------------------------------------------------
            // unary or binary
            default:
            CPPAD_ASSERT_UNKNOWN((n_arg == 1) | (n_arg == 2));
            //
            graph_obj.operator_vec_push_back( graph_op );
            for(size_t i = 0; i < n_arg; ++i)
                graph_obj.operator_arg_push_back( node_arg[i] );
            break;
        }
        i_arg  += n_arg;
    }
    // ----------------------------------------------------------------------
    // variable operators
    pod_vector<size_t> var2node(n_variable);
    var2node[0] = 0; // invalide node value
    for(size_t i = 1; i <= n_variable_ind; ++i)
        var2node[i] = n_dynamic_ind + i;
    for(size_t i = n_variable_ind + 1; i < n_variable; ++i)
        var2node[i] = 0; // invalid node value
    //
    local::play::const_sequential_iterator itr  = play_.begin();
    local::OpCode      var_op;
    const              addr_t* arg;
    size_t             i_var;
    pod_vector<bool>   is_var(2);
    vector<size_t>     atom_node_arg;
    bool in_atomic_call = false;
    bool more_operators = true;
    while(more_operators)
    {   // if non-zero, this is a fixed size operator with this many arguments
        // and implemented after the switch. In additionk, is_var is set for
        // each of the at most 2 arguments.
        size_t fixed_n_arg = 0;

        // invalid value
        graph_op_enum graph_op = n_graph_op;

        // next op
        (++itr).op_info(var_op, arg, i_var);


        // -------------------------------------------------------------------
        // Cases with fixed number of arguments, one or two arguments, and
        // operator is not ignored.
        // -------------------------------------------------------------------
        switch( var_op )
        {   // 2DO: some of these cases can be joined

            // -------------------------------------------------------------
            // unary operators
            case local::AbsOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::AcoshOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::AsinhOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::AtanhOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::ErfOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::ErfcOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::Expm1Op:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::Log1pOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::AcosOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::AsinOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::AtanOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::CoshOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::CosOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::ExpOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::LogOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::NegOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::SignOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::SinhOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::SinOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::SqrtOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::TanhOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            case local::TanOp:
            fixed_n_arg = 1;
            is_var[0] = true;
            break;

            // ---------------------------------------------------------------
            // binary operators
            // ---------------------------------------------------------------

            // first argument a parameter, second argument a variable
            case local::AddpvOp:
            case local::DivpvOp:
            case local::MulpvOp:
            case local::PowpvOp:
            case local::SubpvOp:
            case local::ZmulpvOp:
            fixed_n_arg = 2;
            is_var[0]   = false;
            is_var[1]   = true;
            break;

            // first argument a variable, second argument a parameter
            case local::DivvpOp:
            case local::PowvpOp:
            case local::SubvpOp:
            case local::ZmulvpOp:
            fixed_n_arg = 2;
            is_var[0]   = true;
            is_var[1]   = false;
            break;

            // first argument a variable, second argument a variable
            case local::AddvvOp:
            case local::DivvvOp:
            case local::MulvvOp:
            case local::PowvvOp:
            case local::SubvvOp:
            case local::ZmulvvOp:
            fixed_n_arg = 2;
            is_var[0]   = true;
            is_var[1]   = true;
            break;

            // --------------------------------------------------------------
            default:
            break;
        }
        if( fixed_n_arg > 0 )
        {   // Set graph_op
            switch( var_op )
            {
                // ----------------------------------------------------------
                // unary operators

                case local::AbsOp:
                graph_op = abs_graph_op;
                break;

                case local::AcoshOp:
                graph_op = acosh_graph_op;
                break;

                case local::AsinhOp:
                graph_op = asinh_graph_op;
                break;

                case local::AtanhOp:
                graph_op = atanh_graph_op;
                break;

                case local::ErfOp:
                graph_op = erf_graph_op;
                break;

                case local::ErfcOp:
                graph_op = erfc_graph_op;
                break;

                case local::Expm1Op:
                graph_op = expm1_graph_op;
                break;

                case local::Log1pOp:
                graph_op = log1p_graph_op;
                break;

                case local::AcosOp:
                graph_op = acos_graph_op;
                break;

                case local::AsinOp:
                graph_op = asin_graph_op;
                break;

                case local::AtanOp:
                graph_op = atan_graph_op;
                break;

                case local::CoshOp:
                graph_op = cosh_graph_op;
                break;

                case local::CosOp:
                graph_op = cos_graph_op;
                break;

                case local::ExpOp:
                graph_op = exp_graph_op;
                break;

                case local::LogOp:
                graph_op = log_graph_op;
                break;

                case local::NegOp:
                graph_op = neg_graph_op;
                break;

                case local::SignOp:
                graph_op = sign_graph_op;
                break;

                case local::SinhOp:
                graph_op = sinh_graph_op;
                break;

                case local::SinOp:
                graph_op = sin_graph_op;
                break;

                case local::SqrtOp:
                graph_op = sqrt_graph_op;
                break;

                case local::TanhOp:
                graph_op = tanh_graph_op;
                break;

                case local::TanOp:
                graph_op = tan_graph_op;
                break;

                // -----------------------------------------------------------
                // binary operators

                case local::AddpvOp:
                case local::AddvvOp:
                graph_op = add_graph_op;
                break;

                case local::DivpvOp:
                case local::DivvpOp:
                case local::DivvvOp:
                graph_op = div_graph_op;
                break;

                case local::MulpvOp:
                case local::MulvvOp:
                graph_op = mul_graph_op;
                break;

                case local::PowpvOp:
                case local::PowvpOp:
                case local::PowvvOp:
                graph_op = pow_graph_op;
                break;

                case local::SubpvOp:
                case local::SubvpOp:
                case local::SubvvOp:
                graph_op = sub_graph_op;
                break;

                case local::ZmulpvOp:
                case local::ZmulvpOp:
                case local::ZmulvvOp:
                graph_op = azmul_graph_op;
                break;

                // -----------------------------------------------------------
                default:
                // This should be one of the cases above
                CPPAD_ASSERT_UNKNOWN(false);
                break;
            }
            //
            // var2node and previous_node for this operator
            var2node[i_var] = ++previous_node;
            //
            //
            graph_obj.operator_vec_push_back( graph_op );
            for(size_t i = 0; i < fixed_n_arg; ++i)
            {   if( is_var[i] )
                    graph_obj.operator_arg_push_back( var2node[ arg[i] ] );
                else
                    graph_obj.operator_arg_push_back( par2node[ arg[i] ] );
            }
        }
        // -------------------------------------------------------------------
        // Other cases
        // -------------------------------------------------------------------
        else switch( var_op )
        {
            // -------------------------------------------------------------
            // comparison operators
            case local::EqppOp:
            case local::EqpvOp:
            case local::EqvvOp:
            case local::NeppOp:
            case local::NepvOp:
            case local::NevvOp:
            case local::LtppOp:
            case local::LtpvOp:
            case local::LtvpOp:
            case local::LtvvOp:
            case local::LeppOp:
            case local::LepvOp:
            case local::LevpOp:
            case local::LevvOp:
            {   // node_0, node_1
                size_t node_0, node_1;
                switch( var_op )
                {   // both nodes parameters
                    case local::EqppOp:
                    case local::NeppOp:
                    case local::LtppOp:
                    case local::LeppOp:
                    node_0 = par2node[arg[0]];
                    node_1 = par2node[arg[1]];
                    break;

                    // first node parameter, second variable
                    case local::EqpvOp:
                    case local::NepvOp:
                    case local::LtpvOp:
                    case local::LepvOp:
                    node_0 = par2node[arg[0]];
                    node_1 = var2node[arg[1]];
                    break;

                    // first node variable, second parameter
                    case local::LtvpOp:
                    case local::LevpOp:
                    node_0 = var2node[arg[0]];
                    node_1 = par2node[arg[1]];
                    break;

                    // both nodes variables
                    case local::EqvvOp:
                    case local::NevvOp:
                    case local::LtvvOp:
                    case local::LevvOp:
                    node_0 = var2node[arg[0]];
                    node_1 = var2node[arg[1]];
                    break;

                    // should never get here
                    default:
                    CPPAD_ASSERT_UNKNOWN(false);
                    node_0 = 0; // to avoid compiler warning
                    node_1 = 0;
                    break;
                }
                // Set graph_op
                switch( var_op )
                {
                    case local::EqppOp:
                    case local::EqpvOp:
                    case local::EqvvOp:
                    graph_op = comp_eq_graph_op;
                    break;

                    case local::NeppOp:
                    case local::NepvOp:
                    case local::NevvOp:
                    graph_op = comp_ne_graph_op;
                    break;

                    case local::LtppOp:
                    case local::LtpvOp:
                    case local::LtvpOp:
                    case local::LtvvOp:
                    graph_op = comp_lt_graph_op;
                    break;

                    case local::LeppOp:
                    case local::LepvOp:
                    case local::LevpOp:
                    case local::LevvOp:
                    graph_op = comp_le_graph_op;
                    break;

                    // should never get here
                    default:
                    CPPAD_ASSERT_UNKNOWN(false);
                    graph_op = n_graph_op; // invalid values
                    break;
                }
                graph_obj.operator_vec_push_back( graph_op );
                graph_obj.operator_arg_push_back( node_0 );
                graph_obj.operator_arg_push_back( node_1 );
            }
            break;

            // --------------------------------------------------------------
            // CSumOp
            case local::CSumOp:
            {   // does this case have subtraction terms
                bool has_subtract = (arg[1] != arg[2]) | (arg[3] != arg[4]);
                //
                // var2node for this operator
                if( has_subtract )
                {   // two cumulative sum and one subtract operators
                    var2node[i_var] = previous_node + 3;
                }
                else
                {   // one cumulative sum operator
                    var2node[i_var] = previous_node + 1;
                }
                //
                // previous_node + 1 = sum corresponding to addition terms
                //
                graph_op = sum_graph_op;
                CPPAD_ASSERT_UNKNOWN( 5 <= arg[1] );
                CPPAD_ASSERT_UNKNOWN( arg[2] <= arg[3] );
                size_t n_arg = size_t(1 + arg[1] - 5 + arg[3] - arg[2]);
                //
                // n_arg comes befrore first_node
                graph_obj.operator_arg_push_back(n_arg);
                //
                // graph_op for addition terms
                graph_obj.operator_vec_push_back( graph_op );
                //
                // argument nodes
                size_t arg_node  = par2node[ arg[0] ];
                graph_obj.operator_arg_push_back( arg_node );
                size_t j_arg = 1;
                for(addr_t i = 5; i < arg[1]; ++i)
                {   arg_node    = var2node[ arg[i] ];
                    CPPAD_ASSERT_UNKNOWN( arg_node > 0 );
                    graph_obj.operator_arg_push_back( arg_node );
                    ++j_arg;
                }
                for(addr_t i = arg[2]; i < arg[3]; ++i)
                {   arg_node  = par2node[ arg[i] ];
                    CPPAD_ASSERT_UNKNOWN( arg_node > 0 );
                    graph_obj.operator_arg_push_back( arg_node );
                    ++j_arg;
                }
                CPPAD_ASSERT_UNKNOWN( j_arg == n_arg );
                if( has_subtract )
                {   // previous_node + 2 = sum corresponding to subtract terms
                    CPPAD_ASSERT_UNKNOWN( arg[1] <= arg[2] );
                    CPPAD_ASSERT_UNKNOWN( arg[3] <= arg[4] );
                    n_arg = size_t(arg[2] - arg[1] + arg[4] - arg[3]);
                    //
                    // n_arg comes before first_node
                    graph_obj.operator_arg_push_back(n_arg);
                    //
                    // graph_op for subtraction terms
                    graph_op = sum_graph_op;
                    graph_obj.operator_vec_push_back( graph_op );
                    //
                    // argument nodes
                    j_arg = 0;
                    for(addr_t i = arg[1]; i < arg[2]; ++i)
                    {   arg_node    = var2node[ arg[i] ];
                        CPPAD_ASSERT_UNKNOWN( arg_node > 0 );
                        graph_obj.operator_arg_push_back( arg_node );
                        ++j_arg;
                    }
                    for(addr_t i = arg[3]; i < arg[4]; ++i)
                    {   arg_node  = par2node[ arg[i] ];
                        CPPAD_ASSERT_UNKNOWN( arg_node > 0 );
                        graph_obj.operator_arg_push_back( arg_node );
                        ++j_arg;
                    }
                    CPPAD_ASSERT_UNKNOWN( j_arg == n_arg );
                    //
                    // previous_node + 3 = first sum minus second sum
                    graph_op = sub_graph_op;
                    graph_obj.operator_vec_push_back( graph_op );
                    graph_obj.operator_arg_push_back( previous_node + 1 );
                    graph_obj.operator_arg_push_back( previous_node + 2 );
                }
                // previous node
                if( has_subtract )
                    previous_node += 3;
                else
                    previous_node += 1;
            }
            itr.correct_before_increment();
            break;

            // --------------------------------------------------------------
            case local::DisOp:
            {   // discrete function index
                size_t discrete_index = size_t( arg[0] );
                // name of this discrete function
                std::string name  = discrete<Base>::name( discrete_index );
                //
                // set graph index for this discrete function call
                size_t name_index = graph_obj.discrete_name_vec_find(name);
                if( name_index == graph_obj.discrete_name_vec_size() )
                    graph_obj.discrete_name_vec_push_back(name);
                //
                graph_op = discrete_graph_op;
                graph_obj.operator_vec_push_back( graph_op );
                graph_obj.operator_arg_push_back( name_index );
                graph_obj.operator_arg_push_back( var2node[arg[1]] );
                //
                var2node[i_var] = ++previous_node;
            }
            break;
            // --------------------------------------------------------------
            case local::PriOp:
            {
                // before
                std::string before( play_.GetTxt( size_t(arg[2]) ) );
                size_t before_index = graph_obj.print_text_vec_find(before);
                if( before_index == graph_obj.print_text_vec_size() )
                    graph_obj.print_text_vec_push_back(before);
                // after
                std::string after( play_.GetTxt( size_t(arg[4]) ) );
                size_t after_index = graph_obj.print_text_vec_find(after);
                if( after_index == graph_obj.print_text_vec_size() )
                    graph_obj.print_text_vec_push_back(after);
                // notpos
                size_t notpos_node;
                if( arg[0] & 1 )
                    notpos_node = var2node[ arg[1] ];
                else
                    notpos_node = par2node[ arg[1] ];
                // value
                size_t value_node;
                if( arg[0] & 1 )
                    value_node = var2node[ arg[3] ];
                else
                    value_node = par2node[ arg[3] ];
                //
                graph_op = print_graph_op;
                graph_obj.operator_vec_push_back( graph_op );
                graph_obj.operator_arg_push_back( before_index );
                graph_obj.operator_arg_push_back( after_index );
                graph_obj.operator_arg_push_back( notpos_node );
                graph_obj.operator_arg_push_back( value_node );
            }
            break;

            // --------------------------------------------------------------
            case local::FunapOp:
            atom_node_arg.push_back( par2node[arg[0]] );
            break;

            case local::FunavOp:
            CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) <= i_var );
            atom_node_arg.push_back( var2node[arg[0]] );
            break;

            case local::FunrpOp:
            par2node[arg[0]] = ++previous_node;
            break;

            case local::FunrvOp:
            var2node[i_var] = ++previous_node;
            break;

            case local::AFunOp:
            in_atomic_call = ! in_atomic_call;
            if( in_atomic_call )
            {   atom_node_arg.resize(0);
            }
            else
            {   // This is the AFunOp at the end of the call
                size_t atom_index   = size_t( arg[0] );
                size_t n_arg        = size_t( arg[2] );
                size_t n_result     = size_t( arg[3] );
                CPPAD_ASSERT_UNKNOWN( atom_node_arg.size() == n_arg );
                //
                // get the name for this atomic function
                std::string     name;
                {   bool        set_null = false;
                    size_t      type;
                    void*       ptr;
                    CppAD::local::atomic_index<RecBase>(
                        set_null, atom_index, type, &name, ptr
                    );
                }
                // set graph index for this atomic function
                size_t name_index = graph_obj.atomic_name_vec_find(name);
                if( name_index == graph_obj.atomic_name_vec_size() )
                    graph_obj.atomic_name_vec_push_back(name);
                //
                // for atom_graph_op:
                // name_index, n_result, n_arg come before first_node
                graph_obj.operator_arg_push_back(name_index);
                graph_obj.operator_arg_push_back(n_result);
                graph_obj.operator_arg_push_back(n_arg);
                //
                graph_op = atom_graph_op;
                graph_obj.operator_vec_push_back( graph_op );
                for(size_t i = 0; i < n_arg; ++i)
                    graph_obj.operator_arg_push_back( atom_node_arg[i] );
            }
            break;
            // --------------------------------------------------------------
            // CExpOp:
            case local::CExpOp:
            {   CompareOp cop = CompareOp( arg[0] );
                size_t left, right, if_true, if_false;
                if( arg[1] & 1 )
                    left = var2node[ arg[2] ];
                else
                    left = par2node[ arg[2] ];
                if( arg[1] & 2 )
                    right = var2node[ arg[3] ];
                else
                    right = par2node[ arg[3] ];
                if( arg[1] & 4 )
                    if_true = var2node[ arg[4] ];
                else
                    if_true = par2node[ arg[4] ];
                if( arg[1] & 8 )
                    if_false = var2node[ arg[5] ];
                else
                    if_false = par2node[ arg[5] ];
                switch( cop )
                {   case CompareLt:
                    graph_op = cexp_lt_graph_op;
                    break;

                    case CompareLe:
                    graph_op = cexp_le_graph_op;
                    break;

                    case CompareEq:
                    graph_op = cexp_eq_graph_op;
                    break;

                    case CompareGe:
                    graph_op = cexp_lt_graph_op;
                    std::swap(if_true, if_false);
                    break;

                    case CompareGt:
                    graph_op = cexp_le_graph_op;
                    std::swap(if_true, if_false);
                    break;

                    case CompareNe:
                    graph_op = cexp_eq_graph_op;
                    std::swap(if_true, if_false);
                    break;

                    default:
                    CPPAD_ASSERT_UNKNOWN(false);
                    break;
                }
                // var2node and previous_node for this operator
                var2node[i_var] = ++previous_node;
                //
                graph_obj.operator_vec_push_back( graph_op );
                graph_obj.operator_arg_push_back( left );
                graph_obj.operator_arg_push_back( right );
                graph_obj.operator_arg_push_back( if_true );
                graph_obj.operator_arg_push_back( if_false );
            }
            break;

            // --------------------------------------------------------------
            // EndOp:
            case local::EndOp:
            more_operators = false;
            break;

            // --------------------------------------------------------------
            // InvOp: independent variables
            case local::InvOp:
            // no graph operators for independent variables
            break;

            // --------------------------------------------------------------
            // ParOp:
            case local::ParOp:
            // no need for a graph operator, just map variable to parameter
            var2node[i_var] = par2node[arg[0]];
            break;

            // --------------------------------------------------------------
            default:
            // This error should have been reported above
            CPPAD_ASSERT_UNKNOWN(false);
            break;
        }
    }
    // ----------------------------------------------------------------------
    // output: dependent_vec
    size_t n_dependent = dep_taddr_.size();
    for(size_t i = 0; i < n_dependent; ++i)
        graph_obj.dependent_vec_push_back( var2node[ dep_taddr_[i] ] );
    //
    return;
}

# endif
