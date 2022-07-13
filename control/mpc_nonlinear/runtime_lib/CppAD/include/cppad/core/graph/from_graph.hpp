# ifndef CPPAD_CORE_GRAPH_FROM_GRAPH_HPP
# define CPPAD_CORE_GRAPH_FROM_GRAPH_HPP
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
# include <cppad/core/ad_type.hpp>
# include <cppad/core/discrete/discrete.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*
$begin from_graph$$
$spell
    CppAD
    ind
    vec
    arg
    obj
    op_enum
    dyn2var
$$

$section ADFun Object Corresponding to a CppAD Graph$$

$head Syntax$$
$codei%
    cpp_graph %graph_obj%
    ADFun<%Base%> %fun%
    %fun%.from_graph(%graph_obj%)
    %fun%.from_graph(%graph_obj%, %dyn2var%, %var2dyn%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_ONE_ARGUMENT%// END_ONE_ARGUMENT%1
%$$
$srcthisfile%
    0%// BEGIN_WITH_IS_DYNAMIC%// END_WITH_IS_DYNAMIC%1
%$$

$head Base$$
is the type corresponding to this $cref/ADFun/adfun/$$ object;
i.e., its calculations are done using the type $icode Base$$.

$head RecBase$$
in the prototype above, $icode RecBase$$ is the same type as $icode Base$$.

$head graph_obj$$
is a $cref cpp_ad_graph$$ representation of this function.

$head dyn2var$$
is a vector with size equal to the number of independent dynamic parameters
in the graph; i.e., the size of $cref/p/cpp_ad_graph/Node Indices/p/$$.
It specifies which independent dynamic parameters in the graph are
independent variables in the function $icode fun$$.

$head var2dyn$$
is a vector with size equal to the number of independent variables
in the graph; i.e., the size of $cref/x/cpp_ad_graph/Node Indices/x/$$.
It specifies which independent variables in the graph are
independent dynamic parameters in the function $icode fun$$.

$head fun$$
It $icode dyn2var$$ and $icode var2dyn$$ are not present,
the independent dynamic parameters and independent variables in $icode fun$$
are the same as for the graph.
Otherwise, they are described below.

$subhead m_true, m_false$$
Let $icode m_true$$ ($icode m_false$$) be the number of true (false)
elements of $icode dyn2var$$.

$subhead n_true, n_false$$
Let $icode n_true$$ ($icode n_false$$) be the number of true (false)
elements of $icode var2dyn$$.

$subhead Independent Dynamic Parameters$$
The first $icode m_false$$ independent dynamic parameters in $icode fun$$
correspond to the false components of $icode dyn2var$$
and have the same order as in the graph.
The next $icode n_true$$ independent dynamic parameters in $icode fun$$
correspond to the true components of $icode var2dyn$$
and have the same order as in the graph.

$subhead Independent Variables$$
The first $icode m_true$$ independent variables in $icode fun$$
correspond to the true components of $icode dyn2var$$
and have the same order as in the graph.
The next $icode n_false$$ independent variables in $icode fun$$
correspond to the false components of $icode var2dyn$$
and have the same order as in the graph.

$children%
    example/graph/switch_var_dyn.cpp
%$$
$head Examples$$
The file $cref switch_var_dyn.cpp$$ contains an example and test
of this routine.
For simpler examples, that do not change the dynamic parameters and variables;
see $cref/graph_op_enum examples/graph_op_enum/Examples/$$.

$end
*/
// BEGIN_WITH_IS_DYNAMIC
template <class Base, class RecBase>
void CppAD::ADFun<Base,RecBase>::from_graph(
        const CppAD::cpp_graph& graph_obj     ,
        const CppAD::vector<bool>& dyn2var    ,
        const CppAD::vector<bool>& var2dyn    )
// END_WITH_IS_DYNAMIC
{   using CppAD::isnan;
    using namespace CppAD::graph;
    //
    // some sizes
    const std::string function_name  = graph_obj.function_name_get();
    const size_t n_dynamic_ind       = graph_obj.n_dynamic_ind_get();
    const size_t n_variable_ind      = graph_obj.n_variable_ind_get();
    const size_t n_constant          = graph_obj.constant_vec_size();
    const size_t n_usage             = graph_obj.operator_vec_size();
    const size_t n_dependent         = graph_obj.dependent_vec_size();
    //
    // n_dynamic_ind_fun
    // n_variable_ind_fun
    CPPAD_ASSERT_KNOWN(
        n_variable_ind == var2dyn.size(),
        "from_graph: size of var2dyn not equal "
        "number of independent variables in graph"
    );
    CPPAD_ASSERT_KNOWN(
        n_dynamic_ind == dyn2var.size(),
        "from_graph: size of dyn2val not equal "
        "number of independent dynamic parameters in graph"
    );
    size_t n_dynamic_ind_fun  = 0;
    size_t n_variable_ind_fun = 0;
    for(size_t i = 0; i < n_dynamic_ind; ++i)
    {   if( dyn2var[i] )
            ++n_variable_ind_fun;
        else
            ++n_dynamic_ind_fun;
    }
    for(size_t i = 0; i < n_variable_ind; ++i)
    {   if( var2dyn[i] )
            ++n_dynamic_ind_fun;
        else
            ++n_variable_ind_fun;
    }
    //
    // Start of node indices
# ifndef NDEBUG
    size_t start_dynamic_ind = 1;
    size_t start_independent = start_dynamic_ind + n_dynamic_ind;
    size_t start_constant    = start_independent + n_variable_ind;
    size_t start_operator    = start_constant    + n_constant;
# endif
    //
    // initialize mappings from node index as empty
    // (there is no node zero)
    vector<ad_type_enum>        node_type( 1 );
    local::pod_vector<addr_t>   node2fun( 1 );
    node_type[0] = number_ad_type_enum; // invalid value
    node2fun[0]  = 0;                   // invalid value
    //
    // discrete_index
    // mapping from index in discrete_name_vec to discrete index
    size_t n_list_discrete  = discrete<Base>::list_size();
    size_t n_graph_discrete = graph_obj.discrete_name_vec_size();
    vector<size_t> discrete_index( n_graph_discrete );
    for(size_t i = 0; i < n_graph_discrete; ++i)
        discrete_index[i] = n_list_discrete; // invalid discrete index
    for(size_t index = 0; index < n_list_discrete; ++index)
    {   const char* name( discrete<Base>::name(index) );
        size_t graph_index = graph_obj.discrete_name_vec_find(name);
        if( graph_index != n_graph_discrete )
        {   if( discrete_index[graph_index] != n_list_discrete )
            {   std::string msg = "from_graph: error in call to ";
                msg += name;
                msg += ".\nThere is mor than one discrete ";
                msg += "function with this name";
                //
                // use this source code as point of detection
                bool known       = true;
                int  line        = __LINE__;
                const char* file = __FILE__;
                const char* exp  = "discrete_index[i] == n_list_discrete";
                //
                // CppAD error handler
                ErrorHandler::Call( known, line, file, exp, msg.c_str() );
            }
            discrete_index[graph_index] = index;
        }
    }
    //
    // atomic_three_index
    // mapping from index in atomic_name_vec to atomic three index
    size_t n_graph_atomic = graph_obj.atomic_name_vec_size();
    vector<size_t> atomic_three_index( n_graph_atomic );
    for(size_t index = 0; index < n_graph_atomic; ++index)
        atomic_three_index[index] = 0; // invalid atomic index

    {   bool        set_null = true;
        size_t      index_in = 0;
        size_t      type;
        std::string name;
        void*       ptr;
        size_t n_atomic = CppAD::local::atomic_index<RecBase>(
            set_null, index_in, type, &name, ptr
        );
        set_null = false;
        for(index_in = 1; index_in <= n_atomic; ++index_in)
        {   CppAD::local::atomic_index<RecBase>(
                set_null, index_in, type, &name, ptr
            );
            if( type == 3 )
            {   size_t graph_index = graph_obj.atomic_name_vec_find(name);
                if( graph_index != n_graph_atomic )
                {   if( atomic_three_index[graph_index] != 0 )
                    {   std::string msg = "from_graph: error in call to ";
                        msg += name + ".\n";
                        msg += "There is more than one atomic_three ";
                        msg += "function with this name";
                        //
                        // use this source code as point of detection
                        bool known       = true;
                        int  line        = __LINE__;
                        const char* file = __FILE__;
                        const char* exp  = "atomic_index[index] == 0";
                        //
                        // CppAD error handler
                        ErrorHandler::Call(
                            known, line, file, exp, msg.c_str()
                        );
                    }
                    atomic_three_index[graph_index] = index_in;
                }
            }
        }
    }
    // ----------------------------------------------------------------------
    // Create a recording for this function
    // ----------------------------------------------------------------------

    // start a recording
    local::recorder<Base> rec;
    CPPAD_ASSERT_UNKNOWN( rec.num_op_rec() == 0 );
    rec.set_num_dynamic_ind(n_dynamic_ind_fun);
    rec.set_abort_op_index(0);
    rec.set_record_compare(false);

    // rec_text_index
    // mapping from print_text_vec index to recording index
    vector<addr_t> rec_text_index( graph_obj.print_text_vec_size() );
    for(size_t i = 0; i < graph_obj.print_text_vec_size(); ++i)
    {   const std::string& text = graph_obj.print_text_vec_get(i);
        rec_text_index[i]       = rec.PutTxt( text.c_str() );
    }

    // nan
    Base nan = CppAD::numeric_limits<Base>::quiet_NaN();

    // Place the parameter with index 0 in the tape
    const local::pod_vector_maybe<Base>& parameter( rec.all_par_vec());
    CPPAD_ASSERT_UNKNOWN( parameter.size() == 0 );
    addr_t i_par = rec.put_con_par(nan);
    CPPAD_ASSERT_UNKNOWN( i_par == 0 );
    CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_par] ) );
    //
    // Place the variable with index 0 in the tape
    CPPAD_ASSERT_NARG_NRES(local::BeginOp, 1, 1);
    rec.PutOp(local::BeginOp);
    rec.PutArg(0);
    //
    // Next come the independent dynamic parameters in the graph
    addr_t i_var = 0;
    for(size_t i = 0; i < n_dynamic_ind; ++i)
    {
        if( dyn2var[i] )
        {   i_var = rec.PutOp( local::InvOp );
            node_type.push_back(variable_enum);;
            node2fun.push_back(i_var);
        }
       else
        {   i_par = rec.put_dyn_par(nan, local::ind_dyn );
            CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_par] ) );
            node_type.push_back(dynamic_enum);
            node2fun.push_back(i_par);
        }
    }

    // Next come the independent variables in the graph
    CPPAD_ASSERT_NARG_NRES(local::InvOp, 0, 1);
    for(size_t i = 0; i < n_variable_ind; ++i)
    {   if( var2dyn[i] )
        {   i_par = rec.put_dyn_par(nan, local::ind_dyn );
            CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_par] ) );
            node_type.push_back(dynamic_enum);
            node2fun.push_back(i_par);
        }
        else
        {   i_var = rec.PutOp( local::InvOp );
            node_type.push_back(variable_enum);;
            node2fun.push_back(i_var);
        }
    }
    CPPAD_ASSERT_UNKNOWN( size_t( i_par ) == n_dynamic_ind_fun );
    CPPAD_ASSERT_UNKNOWN( size_t( i_var ) == n_variable_ind_fun );

    // Next come the constant parameters
    for(size_t i = 0; i < n_constant; ++i)
    {   Base par = Base( graph_obj.constant_vec_get(i) );
        i_par = rec.put_con_par(par);
        CPPAD_ASSERT_UNKNOWN( parameter[i_par] == par );
        //
        node_type.push_back(constant_enum);;
        node2fun.push_back(i_par);
    }

    //
    // local arrays used to avoid reallocating memory
    local::pod_vector<addr_t>       temporary;
    vector<ad_type_enum>            type_x;
    vector<addr_t>                  arg;
    vector<size_t>                  arg_node;
    //
    // arrays only used by atom_graph_op
    vector<Base>                    parameter_x, taylor_y;
    vector<ad_type_enum>            type_y;
    vector< AD<Base> >              ax, ay;
    //
    // define here because not using as loop index
    cpp_graph::const_iterator       graph_itr;
    //
    // loop over operators in the recording
# ifndef NDEBUG
    size_t start_result = start_operator;
# endif
    for(size_t op_index = 0; op_index < n_usage; ++op_index)
    {   // op_enum, str_index, n_result, arg_node
        if( op_index == 0 )
            graph_itr = graph_obj.begin();
        else
            ++graph_itr;
        cpp_graph::const_iterator::value_type itr_value = *graph_itr;
        const vector<size_t>& str_index(*itr_value.str_index_ptr );
        graph_op_enum op_enum    = itr_value.op_enum;
        size_t        n_result   = itr_value.n_result;
        size_t        n_arg      = itr_value.arg_node_ptr->size();
        arg.resize(n_arg);
        //
        // make sure type_x is large enough
        type_x.resize(n_arg);
# ifndef NDEBUG
        addr_t n_con_arg      = 0;
# endif
        addr_t n_dyn_arg      = 0;
        addr_t n_var_arg      = 0;
        for(size_t j = 0; j < n_arg; ++j)
        {   size_t node_index = (*itr_value.arg_node_ptr)[j];
            //
            // argument to graph operator
            CPPAD_ASSERT_KNOWN( node_index < start_result,
                "from_graph op argument index is greater or equal\n"
                "the starting index for the next result"
            );
            //
            // type of argument
            type_x[j] = node_type[ node_index ];
            //
            // argument to function operator
            arg[j]  = node2fun[ node_index ];
            CPPAD_ASSERT_UNKNOWN( arg[j] != 0 );
            //
            // count number of arguments of different types
# ifndef NDEBUG
            n_con_arg += addr_t( type_x[j] == constant_enum );
# endif
            n_dyn_arg += addr_t( type_x[j] == dynamic_enum  );
            n_var_arg += addr_t( type_x[j] == variable_enum );
        }
        CPPAD_ASSERT_UNKNOWN(
            n_arg == size_t(n_con_arg + n_dyn_arg + n_var_arg)
        );
        //
        addr_t i_result = 0; // invalid value
        // -------------------------------------------------------------------
        // conditional expressions
        // -------------------------------------------------------------------
        if( op_enum == cexp_eq_graph_op ||
            op_enum == cexp_le_graph_op ||
            op_enum == cexp_lt_graph_op )
        {   CPPAD_ASSERT_UNKNOWN( n_result == 1 && n_arg == 4 );
            // cop
            CompareOp cop;
            if( op_enum == cexp_eq_graph_op )
                cop = CompareEq;
            else if ( op_enum == cexp_le_graph_op )
                cop = CompareLe;
            else
                cop = CompareLt;
            //
            if( n_var_arg == 0 )
            {   if( n_dyn_arg == 0 )
                {   // result is a constant parameter
                    Base result = CondExpOp(cop,
                        parameter[arg[0]],  // left
                        parameter[arg[1]],  // right
                        parameter[arg[2]],  // if_true
                        parameter[arg[3]]   // if_false
                    );
                    i_result = rec.put_con_par(result);
                }
                else
                {   i_result = rec.put_dyn_cond_exp(
                        nan, cop, arg[0], arg[1], arg[2], arg[3]
                    );
                }
            }
            else
            {   // flag marking which arguments are variables
                addr_t flag = 0;
                addr_t bit  = 1;
                for(size_t j = 0; j < 4; ++j)
                {   if( type_x[j] == variable_enum )
                        flag |= bit;
                    bit = 2 * bit;
                }
                CPPAD_ASSERT_UNKNOWN( flag != 0 );
                rec.PutArg(addr_t(cop), flag, arg[0], arg[1], arg[2], arg[3]);
                i_result = rec.PutOp(local::CExpOp);
            }
        }
        // -------------------------------------------------------------------
        // compare operators
        // -------------------------------------------------------------------
        else if(
            op_enum == comp_eq_graph_op ||
            op_enum == comp_le_graph_op ||
            op_enum == comp_lt_graph_op ||
            op_enum == comp_ne_graph_op )
        {   CPPAD_ASSERT_UNKNOWN( n_result == 0 && n_arg == 2 );
            //
            bool var_left  = type_x[0] == variable_enum;
            bool var_right = type_x[1] == variable_enum;
            bool dyn_left  = type_x[0] == dynamic_enum;
            bool dyn_right = type_x[1] == dynamic_enum;
            //
            ax.resize(n_arg);
            // ax[0]
            if( var_left | dyn_left )
                ax[0].taddr_ = arg[0];
            else
                ax[0].value_ = parameter_x[0];
            // ax[1]
            if( var_right | dyn_right )
                ax[1].taddr_ = arg[1];
            else
                ax[1].value_ = parameter_x[1];
            //
            bool result;
            switch( op_enum )
            {
                case comp_eq_graph_op:
                result = true;
                rec.comp_eq(
                var_left, var_right, dyn_left, dyn_right, ax[0], ax[1], result
                );
                break;

                case comp_le_graph_op:
                result = true;
                rec.comp_le(
                var_left, var_right, dyn_left, dyn_right, ax[0], ax[1], result
                );
                break;

                case comp_lt_graph_op:
                result = true;
                rec.comp_lt(
                var_left, var_right, dyn_left, dyn_right, ax[0], ax[1], result
                );
                break;

                case comp_ne_graph_op:
                result = false;
                rec.comp_eq(
                var_left, var_right, dyn_left, dyn_right, ax[0], ax[1], result
                );
                break;


                default:
                CPPAD_ASSERT_UNKNOWN(false);
            }
        }
        // -------------------------------------------------------------------
        // sum operator
        // -------------------------------------------------------------------
        else if( op_enum == sum_graph_op )
        {
            CPPAD_ASSERT_KNOWN( n_result == 1 ,
                "AD graph sum operator: n_result is not 1"
            );
            if( n_var_arg == 0 )
            {   // result of the sum is a parameter
                Base sum_constant = 0.0;
                temporary.resize(0);
                for(size_t j = 0; j < n_arg; j++)
                {   if( type_x[j] == constant_enum )
                        sum_constant += parameter[ arg[j] ];
                    else
                    {   CPPAD_ASSERT_UNKNOWN( type_x[j] == dynamic_enum );
                        temporary.push_back( arg[j] );
                    }
                }
                CPPAD_ASSERT_UNKNOWN( temporary.size() == size_t(n_dyn_arg) );
                //
                // start with constant parameter
                i_result = rec.put_con_par(sum_constant);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == sum_constant );
                //
                // sum the dynamic parameters
                for(addr_t j = 0; j < n_dyn_arg; ++j)
                {   i_result = rec.put_dyn_par(
                        nan, local::add_dyn, i_result, temporary[j]
                    );
                    CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                }
            }
            else
            {   // result of the sum is a variable
                size_t n_temporary = 6 + size_t(n_var_arg + n_dyn_arg);
                if( temporary.size() < n_temporary )
                    temporary.resize( n_temporary );
                Base sum_constant = 0.0;
                addr_t j_variable = 5 ;
                addr_t j_dynamic  = 5 + n_var_arg;
                for(size_t j = 0; j < n_arg; j++)
                {   if( type_x[j] == constant_enum )
                        sum_constant += parameter[ arg[j] ];
                    if( type_x[j] == variable_enum )
                        temporary[ j_variable++ ] = arg[j];
                    if( type_x[j] == dynamic_enum )
                        temporary[ j_dynamic++ ]  = arg[j];
                }
                temporary[j_dynamic] = j_dynamic;
                //
                temporary[0] = rec.put_con_par(sum_constant);
                CPPAD_ASSERT_UNKNOWN(parameter[temporary[0]] == sum_constant);
                //
                temporary[1] = 5 + n_var_arg;
                temporary[2] = 5 + n_var_arg;
                temporary[3] = temporary[2] + n_dyn_arg;
                temporary[4] = temporary[2] + n_dyn_arg;
                //
                i_result = rec.PutOp(local::CSumOp);
                for(size_t j = 0; j < n_temporary; ++j)
                    rec.PutArg( temporary[j] );
                CPPAD_ASSERT_UNKNOWN( local::NumRes(local::CSumOp) == 1 );
            }
        }
        // -------------------------------------------------------------------
        // print operator
        // -------------------------------------------------------------------
        else if( op_enum == print_graph_op )
        {   CPPAD_ASSERT_UNKNOWN( n_arg == 2 && n_result == 0 );
            //
            // before
            size_t before_graph = str_index[0];
            addr_t before_rec   = rec_text_index[before_graph];
            //
            // after
            size_t after_graph  = str_index[1];
            addr_t after_rec    = rec_text_index[after_graph];
            //
            // base 2 representation of [ Var(notpos), Var(value) ]
            addr_t is_var = 0;
            if( type_x[0] == variable_enum )
                is_var += 1;
            if( type_x[1] == variable_enum )
                is_var += 2;
            //
            // record this print operator
            addr_t notpos = arg[0];
            addr_t value  = arg[1];
            rec.PutOp(local::PriOp);
            rec.PutArg(is_var, notpos, before_rec, value, after_rec);
        }
        // -------------------------------------------------------------------
        // discrete operator
        // -------------------------------------------------------------------
        else if( op_enum == discrete_graph_op )
        {   CPPAD_ASSERT_UNKNOWN( n_arg == 1 && n_result == 1 );
            size_t name_index = str_index[0];
            size_t function_index = discrete_index[name_index];
            if( function_index == n_list_discrete )
            {   std::string msg = "from_graph: error in call to ";
                msg += graph_obj.discrete_name_vec_get(name_index);
                msg += ".\nNo previously defined discrete function ";
                msg += "has this name";
                //
                // use this source code as point of detection
                bool known       = true;
                int  line        = __LINE__;
                const char* file = __FILE__;
                const char* exp  =
                    "discrete_index[name_index] != n_list_discrete";
                //
                // CppAD error handler
                ErrorHandler::Call(known, line, file, exp, msg.c_str());
            }
            if( type_x[0] == variable_enum )
            {   CPPAD_ASSERT_NARG_NRES(local::DisOp, 2, 1);
                i_result = rec.PutOp(local::DisOp);
                rec.PutArg( addr_t(function_index) );
                rec.PutArg( arg[0] );
            }
            else if( type_x[0] == dynamic_enum )
            {   i_result = rec.put_dyn_par(
                    nan, local::dis_dyn, addr_t(function_index), arg[0]
                );
            }
            else
            {   Base result = discrete<Base>::eval(
                    function_index, parameter[ arg[0] ]
                );
                i_result = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
            }
        }
        // -------------------------------------------------------------------
        // atomic operator
        // -------------------------------------------------------------------
        else if( op_enum == atom_graph_op )
        {   size_t name_index = str_index[0];
            //
            // atomic_index
            CPPAD_ASSERT_UNKNOWN( name_index < atomic_three_index.size() );
            size_t atomic_index = atomic_three_index[name_index];
            if( atomic_index == 0 )
            {   std::string msg = "from_graph: error in call to ";
                msg += graph_obj.atomic_name_vec_get(name_index);
                msg += ".\n";
                msg += "No previously defined atomic_three function ";
                msg += "has this name";
                //
                // use this source code as point of detection
                bool known       = true;
                int  line        = __LINE__;
                const char* file = __FILE__;
                const char* exp  = "atomic_index != 0";
                //
                // CppAD error handler
                ErrorHandler::Call(known, line, file, exp, msg.c_str());
            }
            //
            // afun
            bool         set_null = false;
            size_t       type;
            std::string* name = nullptr;
            void*        v_ptr;
            CppAD::local::atomic_index<RecBase>(
                set_null, atomic_index, type, name, v_ptr
            );
            CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >( v_ptr );
            //
            // parameter_x
            parameter_x.resize(n_arg);
            for(size_t j = 0; j < n_arg; ++j)
            {   if( type_x[j] == constant_enum )
                    parameter_x[j] = parameter[ arg[j] ];
                else
                    parameter_x[j] = nan;
            }
            //
            // type_y
            type_y.resize(n_result);
            afun->for_type(parameter_x, type_x, type_y);
            //
            // taylor_y
            size_t need_y    = size_t(constant_enum);
            size_t order_low = 0;
            size_t order_up  = 0;
            taylor_y.resize(n_result);
            afun->forward(
                parameter_x ,
                type_x      ,
                need_y      ,
                order_low   ,
                order_up    ,
                parameter_x ,
                taylor_y
            );
            //
            // record_dynamic, record_variable
            bool record_dynamic  = false;
            bool record_variable = false;
            for(size_t i = 0; i < n_result; ++i)
            {   CPPAD_ASSERT_UNKNOWN( type_y[i] <= variable_enum );
                record_dynamic  |= type_y[i] == dynamic_enum;
                record_variable |= type_y[i] == variable_enum;
            }
            // tape_id is zero because not a true recording
            tape_id_t tape_id = 0;
            //
            // ax, ay
            if( record_dynamic || record_variable )
            {   // tape_id (not a recording AD<Base> operations)
                // ax
                ax.resize(n_arg);
                for(size_t j = 0; j < n_arg; ++j)
                {   ax[j].value_ = parameter_x[j];
                    ax[j].taddr_ = arg[j];
                }
                // ay
                ay.resize(n_result);
                for(size_t i = 0; i < n_result; ++i)
                {   ay[i].value_ = taylor_y[i];
                    ay[i].taddr_ = 0;
                }
            }
            if( record_dynamic ) rec.put_dyn_atomic(
                    tape_id, atomic_index, type_x, type_y, ax, ay
            );
            if( record_variable ) rec.put_var_atomic(
                    tape_id, atomic_index, type_x, type_y, ax, ay
            );
            //
            // node_type, node2fun
            for(size_t i = 0; i < n_result; ++i)
            {   node_type.push_back(type_y[i]);
                 switch( type_y[i] )
                {   case constant_enum:
                    node2fun.push_back(rec.put_con_par(taylor_y[i]));
                    break;

                    case dynamic_enum:
                    case variable_enum:
                    node2fun.push_back(ay[i].taddr_);
                    break;

                    default:
                    CPPAD_ASSERT_UNKNOWN(false);
                    break;
                }
            }
        }
        // -------------------------------------------------------------------
        // unary operators
        // -------------------------------------------------------------------
        else if( n_arg == 1 )
        {   CPPAD_ASSERT_UNKNOWN( n_arg == 1 && n_result == 1 );
            Base result; // used in cases argument is a constant
            if( type_x[0] == variable_enum ) switch( op_enum )
            {
                case abs_graph_op:
                i_result = rec.PutOp(local::AbsOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AbsOp) == 1 );
                break;

                case acosh_graph_op:
                i_result = rec.PutOp(local::AcoshOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AcoshOp) == 1 );
                break;

                case asinh_graph_op:
                i_result = rec.PutOp(local::AsinhOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AsinhOp) == 1 );
                break;

                case atanh_graph_op:
                i_result = rec.PutOp(local::AtanhOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AtanhOp) == 1 );
                break;

                case erf_graph_op:
                i_result = rec.PutOp(local::ErfOp);
                CPPAD_ASSERT_UNKNOWN( NumArg(local::ErfOp) == 3 );
                //
                // arg[0] = variable index for function argument
                rec.PutArg( arg[0] );
                //
                // parameter[ arg[1] ] = 0.0
                i_par = rec.put_con_par( Base(0.0) );
                rec.PutArg( i_par );
                //
                // parameter[ arg[2] ] = 2 / sqrt(pi)
                i_par = rec.put_con_par(Base(
                    1.0 / std::sqrt( std::atan(1.0) )
                ));
                rec.PutArg( i_par );
                //
                break;

                case erfc_graph_op:
                i_result = rec.PutOp(local::ErfcOp);
                CPPAD_ASSERT_UNKNOWN( NumArg(local::ErfcOp) == 3 );
                //
                // arg[0] = variable index for function argument
                rec.PutArg( arg[0] );
                //
                // parameter[ arg[1] ] = 0.0
                i_par = rec.put_con_par( Base(0.0) );
                rec.PutArg( i_par );
                //
                // parameter[ arg[2] ] = 2 / sqrt(pi)
                i_par = rec.put_con_par(Base(
                    1.0 / std::sqrt( std::atan(1.0) )
                ));
                rec.PutArg( i_par );
                //
                break;

                case expm1_graph_op:
                i_result = rec.PutOp(local::Expm1Op);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::Expm1Op) == 1 );
                break;

                case log1p_graph_op:
                i_result = rec.PutOp(local::Log1pOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::Log1pOp) == 1 );
                break;

                case acos_graph_op:
                i_result = rec.PutOp(local::AcosOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AcosOp) == 1 );
                break;

                case asin_graph_op:
                i_result = rec.PutOp(local::AsinOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AsinOp) == 1 );
                break;

                case atan_graph_op:
                i_result = rec.PutOp(local::AtanOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AtanOp) == 1 );
                break;

                case cosh_graph_op:
                i_result = rec.PutOp(local::CoshOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::CoshOp) == 1 );
                break;

                case cos_graph_op:
                i_result = rec.PutOp(local::CosOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::CosOp) == 1 );
                break;

                case exp_graph_op:
                i_result = rec.PutOp(local::ExpOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::ExpOp) == 1 );
                break;

                case log_graph_op:
                i_result = rec.PutOp(local::LogOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::LogOp) == 1 );
                break;

                case neg_graph_op:
                i_result = rec.PutOp(local::NegOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::NegOp) == 1 );
                break;

                case sign_graph_op:
                i_result = rec.PutOp(local::SignOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::SignOp) == 1 );
                break;

                case sinh_graph_op:
                i_result = rec.PutOp(local::SinhOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::SinhOp) == 1 );
                break;

                case sin_graph_op:
                i_result = rec.PutOp(local::SinOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::SinOp) == 1 );
                break;

                case sqrt_graph_op:
                i_result = rec.PutOp(local::SqrtOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::SqrtOp) == 1 );
                break;

                case tanh_graph_op:
                i_result = rec.PutOp(local::TanhOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::TanhOp) == 1 );
                break;

                case tan_graph_op:
                i_result = rec.PutOp(local::TanOp);
                rec.PutArg( arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::TanOp) == 1 );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;
            }
            else if( type_x[0] == dynamic_enum ) switch( op_enum )
            {
                case abs_graph_op:
                i_result = rec.put_dyn_par(nan, local::abs_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case acosh_graph_op:
                i_result = rec.put_dyn_par(nan, local::acosh_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case asinh_graph_op:
                i_result = rec.put_dyn_par(nan, local::asinh_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case atanh_graph_op:
                i_result = rec.put_dyn_par(nan, local::atanh_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case erf_graph_op:
                i_result = rec.put_dyn_par(nan, local::erf_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case erfc_graph_op:
                i_result = rec.put_dyn_par(nan, local::erfc_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case expm1_graph_op:
                i_result = rec.put_dyn_par(nan, local::expm1_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case log1p_graph_op:
                i_result = rec.put_dyn_par(nan, local::log1p_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case acos_graph_op:
                i_result = rec.put_dyn_par(nan, local::acos_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case asin_graph_op:
                i_result = rec.put_dyn_par(nan, local::asin_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case atan_graph_op:
                i_result = rec.put_dyn_par(nan, local::atan_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case cosh_graph_op:
                i_result = rec.put_dyn_par(nan, local::cosh_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case cos_graph_op:
                i_result = rec.put_dyn_par(nan, local::cos_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case exp_graph_op:
                i_result = rec.put_dyn_par(nan, local::exp_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case log_graph_op:
                i_result = rec.put_dyn_par(nan, local::log_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case neg_graph_op:
                i_result = rec.put_dyn_par(nan, local::neg_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case sign_graph_op:
                i_result = rec.put_dyn_par(nan, local::sign_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case sinh_graph_op:
                i_result = rec.put_dyn_par(nan, local::sinh_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case sin_graph_op:
                i_result = rec.put_dyn_par(nan, local::sin_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case sqrt_graph_op:
                i_result = rec.put_dyn_par(nan, local::sqrt_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case tanh_graph_op:
                i_result = rec.put_dyn_par(nan, local::tanh_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case tan_graph_op:
                i_result = rec.put_dyn_par(nan, local::tan_dyn, arg[0] );
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;
            }
            else switch( op_enum )
            {
                case abs_graph_op:
                result    = CppAD::abs( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case acosh_graph_op:
                result    = CppAD::acosh( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case asinh_graph_op:
                result    = CppAD::asinh( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case atanh_graph_op:
                result    = CppAD::atanh( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case erf_graph_op:
                result    = CppAD::erf( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case erfc_graph_op:
                result    = CppAD::erfc( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case expm1_graph_op:
                result    = CppAD::expm1( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case log1p_graph_op:
                result    = CppAD::log1p( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case acos_graph_op:
                result    = CppAD::acos( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case asin_graph_op:
                result    = CppAD::asin( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case atan_graph_op:
                result    = CppAD::atan( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case cosh_graph_op:
                result    = CppAD::cosh( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case cos_graph_op:
                result    = CppAD::cos( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case exp_graph_op:
                result    = CppAD::exp( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case log_graph_op:
                result    = CppAD::log( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case neg_graph_op:
                result    = - parameter[ arg[0] ];
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case sign_graph_op:
                result    = CppAD::sign( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case sinh_graph_op:
                result    = CppAD::sinh( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case sin_graph_op:
                result    = CppAD::sin( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case sqrt_graph_op:
                result    = CppAD::sqrt( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case tanh_graph_op:
                result    = CppAD::tanh( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case tan_graph_op:
                result    = CppAD::tan( parameter[ arg[0] ] );
                i_result  = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;
            }
        }
        // -------------------------------------------------------------------
        // binary operators
        // -------------------------------------------------------------------
        else
        {   CPPAD_ASSERT_UNKNOWN( n_arg == 2 && n_result == 1 );
            Base result; // used in cases where both arguments are constants
            if( type_x[0] == variable_enum && type_x[1] == variable_enum )
            switch( op_enum )
            {
                case add_graph_op:
                i_result = rec.PutOp(local::AddvvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AddvvOp) == 2 );
                break;

                case azmul_graph_op:
                i_result = rec.PutOp(local::ZmulvvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::ZmulvvOp) == 2 );
                break;

                case div_graph_op:
                i_result = rec.PutOp(local::DivvvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::DivvvOp) == 2 );
                break;

                case mul_graph_op:
                i_result = rec.PutOp(local::MulvvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::MulvvOp) == 2 );
                break;

                case pow_graph_op:
                i_result = rec.PutOp(local::PowvvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::PowvvOp) == 2 );
                break;

                case sub_graph_op:
                i_result = rec.PutOp(local::SubvvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::SubvvOp) == 2 );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;
            }
            else if( type_x[0] == variable_enum ) switch( op_enum )
            {
                // addition is communitative, so use Addpv
                case add_graph_op:
                i_result = rec.PutOp(local::AddpvOp);
                rec.PutArg( arg[1], arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AddpvOp) == 2 );
                break;

                case azmul_graph_op:
                i_result = rec.PutOp(local::ZmulvpOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::ZmulvpOp) == 2 );
                break;

                case div_graph_op:
                i_result = rec.PutOp(local::DivvpOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::DivvpOp) == 2 );
                break;

                // multiplication is communitative, so use Mulpv
                case mul_graph_op:
                i_result = rec.PutOp(local::MulpvOp);
                rec.PutArg( arg[1], arg[0] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::MulpvOp) == 2 );
                break;

                case pow_graph_op:
                i_result = rec.PutOp(local::PowvpOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::PowvpOp) == 2 );
                break;

                case sub_graph_op:
                i_result = rec.PutOp(local::SubvpOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::SubvpOp) == 2 );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;
            }
            else if( type_x[1] == variable_enum ) switch( op_enum )
            {
                case add_graph_op:
                i_result = rec.PutOp(local::AddpvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::AddpvOp) == 2 );
                break;

                case azmul_graph_op:
                i_result = rec.PutOp(local::ZmulpvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::ZmulpvOp) == 2 );
                break;

                case div_graph_op:
                i_result = rec.PutOp(local::DivpvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::DivpvOp) == 2 );
                break;

                case mul_graph_op:
                i_result = rec.PutOp(local::MulpvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::MulpvOp) == 2 );
                break;

                case pow_graph_op:
                i_result = rec.PutOp(local::PowpvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::PowpvOp) == 2 );
                break;

                case sub_graph_op:
                i_result = rec.PutOp(local::SubpvOp);
                rec.PutArg( arg[0], arg[1] );
                CPPAD_ASSERT_UNKNOWN( NumArg(local::SubpvOp) == 2 );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;
            }
            else if( type_x[0] == dynamic_enum || type_x[1] == dynamic_enum )
            switch( op_enum )
            {
                case add_graph_op:
                i_result =
                    rec.put_dyn_par(nan, local::add_dyn, arg[0], arg[1]);
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case azmul_graph_op:
                i_result =
                    rec.put_dyn_par(nan, local::zmul_dyn, arg[0], arg[1]);
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case div_graph_op:
                i_result =
                    rec.put_dyn_par(nan, local::div_dyn, arg[0], arg[1]);
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case mul_graph_op:
                i_result =
                    rec.put_dyn_par(nan, local::mul_dyn, arg[0], arg[1]);
                break;

                case pow_graph_op:
                i_result =
                    rec.put_dyn_par(nan, local::pow_dyn, arg[0], arg[1]);
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                case sub_graph_op:
                i_result =
                    rec.put_dyn_par(nan, local::sub_dyn, arg[0], arg[1]);
                CPPAD_ASSERT_UNKNOWN( isnan( parameter[i_result] ) );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;
            }
            else switch( op_enum )
            {
                case add_graph_op:
                result = parameter[ arg[0] ] + parameter[ arg[1] ];
                i_result = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case azmul_graph_op:
                result = azmul( parameter[ arg[0] ] , parameter[ arg[1] ] );
                i_result = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case div_graph_op:
                result = parameter[ arg[0] ] / parameter[ arg[1] ];
                i_result = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case mul_graph_op:
                result = parameter[ arg[0] ] * parameter[ arg[1] ];
                i_result = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case pow_graph_op:
                result = pow( parameter[ arg[0] ],  parameter[ arg[1] ] );
                i_result = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                case sub_graph_op:
                result = parameter[ arg[0] ] - parameter[ arg[1] ];
                i_result = rec.put_con_par(result);
                CPPAD_ASSERT_UNKNOWN( parameter[i_result] == result );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( false );
                break;

            }
        }
        // case where node_type and node2fun for the results are set above
        if( op_enum != atom_graph_op && n_result != 0 )
        {   // set node_type and node2fun for result
            //
            CPPAD_ASSERT_UNKNOWN( i_result != 0 );
            CPPAD_ASSERT_UNKNOWN( n_result == 1 );
            if( n_var_arg > 0 )
                node_type.push_back(variable_enum);
            else if( n_dyn_arg > 0 )
                node_type.push_back(dynamic_enum);
            else
                node_type.push_back(constant_enum);
            node2fun.push_back(i_result);
        }
        //
# ifndef NDEBUG
        start_result          += n_result;
# endif
        CPPAD_ASSERT_UNKNOWN( node2fun.size() == start_result );
        CPPAD_ASSERT_UNKNOWN( node_type.size() == start_result );
    }
    // set this->dep_parameter_, set this->dep_taddr_
    //
    CPPAD_ASSERT_NARG_NRES(local::ParOp, 1, 1);
    dep_parameter_.resize( n_dependent );
    dep_taddr_.resize( n_dependent );
    for(size_t i = 0; i < n_dependent; ++i)
    {
        CPPAD_ASSERT_UNKNOWN(
            node_type[ graph_obj.dependent_vec_get(i) ] != number_ad_type_enum
        );
        if( node_type[ graph_obj.dependent_vec_get(i) ] == variable_enum )
        {   dep_parameter_[i] = false;
            dep_taddr_[i]     = size_t( node2fun[ graph_obj.dependent_vec_get(i) ] );
        }
        else
        {   dep_parameter_[i] = true;
            dep_taddr_[i]     = size_t( rec.PutOp(local::ParOp) );
            rec.PutArg( node2fun[ graph_obj.dependent_vec_get(i) ] );
        }
    }
    rec.PutOp(local::EndOp);
    // ----------------------------------------------------------------------
    // End recording, set private member data except for
    // dep__parameter_ and dep_taddr_
    // ----------------------------------------------------------------------
    //
    // bool values in this object except check_for_nan_
    has_been_optimized_        = false;
    //
    // size_t values in this object
    compare_change_count_      = 1;
    compare_change_number_     = 0;
    compare_change_op_index_   = 0;
    num_order_taylor_          = 0;
    cap_order_taylor_          = 0;
    num_direction_taylor_      = 0;
    num_var_tape_              = rec.num_var_rec();
    //
    // taylor_
    taylor_.resize(0);
    //
    // cskip_op_
    cskip_op_.resize( rec.num_op_rec() );
    //
    // load_op2var_
    load_op2var_.resize( rec.num_var_load_rec() );
    //
    // play_
    // Now that each dependent variable has a place in the recording,
    // and there is a EndOp at the end of the record, we can transfer the
    // recording to the player and and erase the recording.
    play_.get_recording(rec, n_variable_ind_fun);
    //
    // ind_taddr_
    // Note that play_ has been set, we can use it to check operators
    ind_taddr_.resize(n_variable_ind_fun);
    CPPAD_ASSERT_UNKNOWN( n_variable_ind_fun < num_var_tape_);
    for(size_t j = 0; j < n_variable_ind_fun; j++)
    {   CPPAD_ASSERT_UNKNOWN( play_.GetOp(j+1) == local::InvOp );
        ind_taddr_[j] = j+1;
    }
    //
    // for_jac_sparse_pack_, for_jac_sparse_set_
    for_jac_sparse_pack_.resize(0, 0);
    for_jac_sparse_set_.resize(0,0);
    //
    // resize subgraph_info_
    subgraph_info_.resize(
        ind_taddr_.size(),   // n_dep
        dep_taddr_.size(),   // n_ind
        play_.num_op_rec(),  // n_op
        play_.num_var_rec()  // n_var
    );
    //
    // set the function name
    function_name_ = function_name;
    //
    return;
}
// BEGIN_ONE_ARGUMENT
template <class Base, class RecBase>
void CppAD::ADFun<Base,RecBase>::from_graph(
    const CppAD::cpp_graph& graph_obj     )
// END_ONE_ARGUMENT
{   size_t n_variable_ind = graph_obj.n_variable_ind_get();
    size_t n_dynamic_ind  = graph_obj.n_dynamic_ind_get();
    CppAD::vector<bool> dyn2var(n_dynamic_ind), var2dyn(n_variable_ind);
    for(size_t j = 0; j < n_dynamic_ind; ++j)
        dyn2var[j] = false;
    for(size_t j = 0; j < n_variable_ind; ++j)
        var2dyn[j] = false;
    //
    from_graph(graph_obj, dyn2var, var2dyn);
}

} // END_CPPAD_NAMESPACE

# endif
