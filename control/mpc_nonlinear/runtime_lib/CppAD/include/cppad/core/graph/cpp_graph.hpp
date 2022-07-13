# ifndef CPPAD_CORE_GRAPH_CPP_GRAPH_HPP
# define CPPAD_CORE_GRAPH_CPP_GRAPH_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <iomanip>
# include <string>
# include <cppad/utility/vector.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>
# include <cppad/local/graph/cpp_graph_itr.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

class cpp_graph { // BEGIN_CPP_GRAPH_CLASS
public:
    typedef CppAD::graph::graph_op_enum graph_op_enum;
private:
    //
    std::string                   function_name_;
    vector<std::string>           discrete_name_vec_;
    vector<std::string>           atomic_name_vec_;
    vector<std::string>           print_text_vec_;
    size_t                        n_dynamic_ind_;
    size_t                        n_variable_ind_;
    vector<double>                constant_vec_;
    vector<graph_op_enum>         operator_vec_;
    vector<size_t>                operator_arg_;
    vector<size_t>                dependent_vec_;
public:
    typedef local::graph::cpp_graph_itr const_iterator;
    //
    const_iterator begin(void) const
    {   size_t op_index = 0;
        return const_iterator(operator_vec_, operator_arg_, op_index);
    }
    const_iterator end(void)
    {   size_t op_index = operator_vec_.size();
        return const_iterator(operator_vec_, operator_arg_, op_index);
    }
/*
-------------------------------------------------------------------------------
$begin cpp_graph_ctor$$
$spell
    obj
    cpp
    ind
    vec
    arg
$$

$section C++ AD Graph Constructor$$

$head Syntax$$
$codei%cpp_graph %graph_obj%
%$$
$icode%graph_obj%.initialize()
%$$

$head function_name$$
$cref/function_name/cpp_ad_graph/function_name/$$
is initialized to the empty string.

$head n_dynamic_ind$$
$cref/n_dynamic_ind/cpp_ad_graph/n_dynamic_ind/$$ is initialized as zero.

$head n_variable_ind$$
$cref/n_variable_ind/cpp_ad_graph/n_variable_ind/$$ is initialized as zero.

$head constant_vec$$
$cref/constant_vec/cpp_ad_graph/constant_vec/$$ is initialized as empty.

$head operator_vec$$
$cref/operator_vec/cpp_ad_graph/operator_vec/$$ is initialized as empty.

$head operator_arg$$
$cref/operator_arg/cpp_ad_graph/operator_arg/$$ is initialized as empty.

$head dependent_vec$$
$cref/dependent_vec/cpp_ad_graph/dependent_vec/$$ is initialized as empty.

$head Parallel Mode$$
The first use of the $code cpp_graph$$ constructor
cannot be in $cref/parallel/ta_in_parallel/$$ execution mode.

$end
--------------------------------------------------------------------------------
*/
public:
    void initialize(void)
    {   function_name_  = "";
        n_dynamic_ind_  = 0;
        n_variable_ind_  = 0;
        discrete_name_vec_.resize(0);
        atomic_name_vec_.resize(0);
        print_text_vec_.resize(0);
        constant_vec_.resize(0);
        operator_vec_.resize(0);
        operator_arg_.resize(0);
        dependent_vec_.resize(0);
        return;
    }
    cpp_graph(void)
    {   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL
        static bool first = true;
        if( first )
        {   first = false;
            CPPAD_ASSERT_UNKNOWN( local::graph::op_name2enum.size() == 0 );
            // initialize cpp_graph global variables in cpp_graph_op.cpp
            local::graph::set_operator_info();
        }
        initialize();
    }
/*
---------------------------------------------------------------------------------
$begin cpp_graph_scalar$$
$spell
    obj
    cpp
    ind
    std
    const
$$

$section C++ AD Graph Scalar Values$$

$head Syntax$$

$subhead Get$$
$icode%function_name% = %graph_obj%.function_name_get()
%$$
$icode%n_dynamic_ind% = %graph_obj%.n_dynamic_ind_get()
%$$
$icode%n_variable_ind% = %graph_obj%.n_variable_ind_get()
%$$

$subhead Set$$
$icode%graph_obj%.function_name_set(%function_name%)
%$$
$icode%graph_obj%.n_dynamic_ind_set(%n_dynamic_ind%)
%$$
$icode%graph_obj%.n_variable_ind_set(%n_variable_ind%)
%$$

$head Set$$
The argument for all the set operations is const.

$head graph_obj$$
is an $code cpp_graph$$ object.
It is const for all the get functions.

$head function_name$$
is a $code std::string&$$ specifying the name of the function
for this graph.

$head n_dynamic_ind$$
is a $code size_t$$ specifying the number of independent dynamic parameters.

$head n_variable_ind$$
is a $code size_t$$ specifying the number of independent variables.

$end
*/
    // function_name
    const std::string& function_name_get(void) const
    {   return function_name_; }
    void function_name_set(const std::string& function_name)
    {   function_name_ = function_name; }
    //
    // n_dynamic_ind
    const size_t& n_dynamic_ind_get(void) const
    {   return n_dynamic_ind_; }
    void n_dynamic_ind_set(const size_t n_dynamic_ind)
    {   n_dynamic_ind_ = n_dynamic_ind; }
    //
    // n_variable_ind
    const size_t& n_variable_ind_get(void) const
    {   return n_variable_ind_; }
    void n_variable_ind_set(const size_t n_variable_ind)
    {   n_variable_ind_ = n_variable_ind; }
/*
---------------------------------------------------------------------------------
$begin cpp_graph_vector$$
$spell
    obj
    cpp
    ind
    std
    const
    vec
    arg
    op_enum
$$

$section C++ AD Graph Vector Values$$

$head Syntax$$

$subhead Size$$
$icode%size% = %graph_obj%.discrete_name_vec_size()
%$$
$icode%size% = %graph_obj%.atomic_name_vec_size()
%$$
$icode%size% = %graph_obj%.print_text_vec_size()
%$$
$icode%size% = %graph_obj%.constant_vec_size()
%$$
$icode%size% = %graph_obj%.operator_vec_size()
%$$
$icode%size% = %graph_obj%.operator_arg_size()
%$$
$icode%size% = %graph_obj%.dependent_vec_size()
%$$

$subhead Get$$
$icode%discrete_name% = %graph_obj%.discrete_name_vec_get(%index%)
%$$
$icode%atomic_name%   = %graph_obj%.atomic_name_vec_get(%index%)
%$$
$icode%print_text%    = %graph_obj%.print_text_vec_get(%index%)
%$$
$icode%constant%      = %graph_obj%.constant_vec_get(%index%)
%$$
$icode%op_enum%       = %graph_obj%.operator_vec_get(%index%)
%$$
$icode%argument%      = %graph_obj%.operator_arg_get(%index%)
%$$
$icode%node_index%    = %graph_obj%.dependent_vec_get(%index%)
%$$

$subhead Push Back$$
$icode%graph_obj%.discrete_name_vec_push_back(%discrete_name%)
%$$
$icode%graph_obj%.atomic_name_vec_push_back(%atomic_name%)
%$$
$icode%graph_obj%.print_text_vec_push_back(%print_text%)
%$$
$icode%graph_obj%.constant_vec_push_back(%constant%)
%$$
$icode%graph_obj%.operator_vec_push_back(%op_enum%)
%$$
$icode%graph_obj%.operator_arg_push_back(%argument%)
%$$
$icode%graph_obj%.dependent_vec_push_back(%node_index%)
%$$

$subhead Find$$
$icode%discrete_index% = %graph_obj%.discrete_name_vec_find(%discrete_name%)
%$$
$icode%atomic_index%   = %graph_obj%.atomic_name_vec_find(%atomic_name%)
%$$
$icode%print_index%    = %graph_obj%.print_text_vec_find(%print_text%)
%$$

$head Arguments$$
All of the member function arguments are either call by value or const.

$head size$$
is a $code size_t$$ value equal to the current size of the specified vector.

$head index$$
is a $code size_t$$ value that must be less than the current size
of the specified vector.

$head push_back$$
The arguments for all the push_back functions are const.
The size of the specified vector before a push_back,
is the index in the vector corresponding to the argument value.
The size of the vector after the push_back is the size before plus one.

$head graph_obj$$
is an $code cpp_graph$$ object.
It is const for the size, get, and find functions and
not const for the push_back functions.

$head discrete_name$$
is a $code std::string$$ equal to the name of a $cref discrete$$ function.

$head atomic_name$$
is a $code std::string$$ equal to the name of an $cref atomic_three$$ function.

$head print_text$$
is a $code std::string$$ equal to the text to be printed.

$head constant$$
is a $code double$$ equal to the constant with the corresponding
index in $code constant_vec$$.

$head op_enum$$
is the $cref graph_op_enum$$ for corresponding operator.

$head argument$$
is the $code size_t$$ value for corresponding operator argument.

$head node_index$$
is the node index for the corresponding dependent variable with
the corresponding index in
$cref/dependent_vec/cpp_ad_graph/dependent_vec/$$.

$head discrete_index$$
is the index such that
$codei%
    %discrete_name% == %graph_obj%.discrete_name_vec_get(%discrete_index%)
%$$
If there is no such index,
$codei%
    %discrete_index% == %graph_obj%.discrete_name_vec_size()
%$$

$head atomic_index$$
is the index such that
$codei%
    %atomic_name% == %graph_obj%.atomic_name_vec_get(%atomic_index%)
%$$
If there is no such index,
$codei%
    %atomic_index% == %graph_obj%.atomic_name_vec_size()
%$$

$head print_index$$
is the index such that
$codei%
    %print_text% == %graph_obj%.print_text_vec_get(%print_index%)
%$$
If there is no such index,
$codei%
    %print_index% == %graph_obj%.print_text_vec_size()
%$$


$end
*/
    // discrete_name_vec
    const std::string& discrete_name_vec_get(size_t index) const
    {   return discrete_name_vec_[index]; }
    size_t discrete_name_vec_size(void) const
    {   return discrete_name_vec_.size(); }
    void discrete_name_vec_push_back(const std::string& discrete_name)
    {   discrete_name_vec_.push_back(discrete_name); }
    size_t discrete_name_vec_find(const std::string& discrete_name) const
    {   for(size_t i = 0; i < discrete_name_vec_.size(); ++i)
            if( discrete_name == discrete_name_vec_[i] )
                return i;
        return discrete_name_vec_.size();
    }
    //
    // atomic_name_vec
    const std::string& atomic_name_vec_get(size_t index) const
    {   return atomic_name_vec_[index]; }
    size_t atomic_name_vec_size(void) const
    {   return atomic_name_vec_.size(); }
    void atomic_name_vec_push_back(const std::string& atomic_name)
    {   atomic_name_vec_.push_back(atomic_name); }
    size_t atomic_name_vec_find(const std::string& atomic_name) const
    {   for(size_t i = 0; i < atomic_name_vec_.size(); ++i)
            if( atomic_name == atomic_name_vec_[i] )
                return i;
        return atomic_name_vec_.size();
    }
    //
    // print_text_vec
    const std::string& print_text_vec_get(size_t index) const
    {   return print_text_vec_[index]; }
    size_t print_text_vec_size(void) const
    {   return print_text_vec_.size(); }
    void print_text_vec_push_back(const std::string& atomic_name)
    {   print_text_vec_.push_back(atomic_name); }
    size_t print_text_vec_find(const std::string& print_text) const
    {   for(size_t i = 0; i < print_text_vec_.size(); ++i)
            if( print_text == print_text_vec_[i] )
                return i;
        return print_text_vec_.size();
    }
    //
    // constant_vec
    const double& constant_vec_get(size_t index) const
    {   return constant_vec_[index]; }
    size_t constant_vec_size(void) const
    {   return constant_vec_.size(); }
    void constant_vec_push_back(const double& constant)
    {   constant_vec_.push_back(constant); }
    //
    // oerator_vec
    const graph_op_enum& operator_vec_get(size_t index) const
    {   return operator_vec_[index]; }
    size_t operator_vec_size(void) const
    {   return operator_vec_.size(); }
    void operator_vec_push_back(const graph_op_enum op_enum)
    {   operator_vec_.push_back(op_enum); }
    //
    // operator_arg
    const size_t& operator_arg_get(size_t index) const
    {   return operator_arg_[index]; }
    size_t operator_arg_size(void) const
    {   return operator_arg_.size(); }
    void operator_arg_push_back(const size_t argument)
    {   operator_arg_.push_back(argument); }
    //
    // dependent_vec
    const size_t& dependent_vec_get(size_t index) const
    {   return dependent_vec_[index]; }
    size_t dependent_vec_size(void) const
    {   return dependent_vec_.size(); }
    void dependent_vec_push_back(const size_t node_index)
    {   dependent_vec_.push_back(node_index); }
/*
$begin cpp_graph_print$$
$spell
    obj
    const
    std::ostream
    cpp
$$

$section Print A C++ AD Graph$$

$head Syntax$$
$icode%graph_obj%.print(%os%)
%$$

$head graph_obj$$
is an const $code cpp_graph$$ object.

$head os$$
Is the $code std::ostream$$ where the graph is printed.

$head Discussion$$
This function is included to help with using the $code cpp_graph$$ class.
The formatting of it's output is not part of the API; i.e.,
it may change in the future.

$children%
    example/graph/print_graph.cpp
%$$
$head Example$$
The file $cref print_graph.cpp$$ contains an example and test of this operation.

$end
*/
    void print(std::ostream& os) const
    {   using std::setw;
        using std::string;
        //
        // function name
        if( function_name_ != "" )
            os << function_name_ << "\n";
        //
        // initialize node index
        size_t node_index = 1;
        //
        // text vector
        size_t n_text = print_text_vec_.size();
        for(size_t i = 0; i < n_text; ++i)
        {   string s_i = "c[" + std::to_string(i) + "]";
            //
            os << setw(11) << "";
            os << setw(10) << s_i;
            os << "'" << print_text_vec_[i] << "'";
            os << "\n";
        }
        //
        //  parameter vector
        for(size_t i = 0; i < n_dynamic_ind_; i++)
        {   string p_i = "p[" + std::to_string(i) + "]";
            //
            os << setw(11)  << node_index;
            os << setw(10) << p_i;
            os << "\n";
            ++node_index;
        }
        //
        //  variable vector
        for(size_t i = 0; i < n_variable_ind_; i++)
        {   string x_i = "x[" + std::to_string(i) + "]";
            //
            os << setw(11)  << node_index;
            os << setw(10) << x_i;
            os << "\n";
            ++node_index;
        }
        //
        //  constant vector
        size_t n_constant = constant_vec_.size();
        for(size_t i = 0; i < n_constant; i++)
        {   string c_i = "c[" + std::to_string(i) + "]";
            //
            os << setw(11) << node_index;
            os << setw(10) << c_i;
            os << setw(20) << constant_vec_[i];
            os << "\n";
            ++node_index;
        }

        size_t                    n_op = operator_vec_.size();
        cpp_graph::const_iterator itr;
        for(size_t op_index = 0; op_index < n_op; ++op_index)
        {   if( op_index == 0 )
                itr = begin();
            else
                ++itr;
            cpp_graph::const_iterator::value_type itr_value = *itr;
            //
            const vector<size_t>& str_index( *itr_value.str_index_ptr );
            const vector<size_t>&       arg( *itr_value.arg_node_ptr );
            graph_op_enum          op_enum  = itr_value.op_enum;
            size_t                n_result  = itr_value.n_result;
            size_t                   n_arg  = arg.size();
            CPPAD_ASSERT_UNKNOWN( n_arg > 0 );
            //
            string op_name = local::graph::op_enum2name[ op_enum ];
            //
            if( n_result == 0 )
                os << setw(11) << "";
            else if( n_result == 1 )
                os << setw(11)  << node_index;
            else
            {   string first = std::to_string(node_index);
                string last  = std::to_string(node_index + n_result - 1);
                os << setw(11) << first + "-" + last;
            }
            os << setw(10) << op_name;
            for(size_t i = 0; i < n_arg; ++i)
                os << setw(5) << arg[i];

            switch( op_enum )
            {
                case graph::discrete_graph_op:
                CPPAD_ASSERT_UNKNOWN( str_index.size() == 1 );
                os << discrete_name_vec_get( str_index[0] );
                break;

                case graph::atom_graph_op:
                CPPAD_ASSERT_UNKNOWN( str_index.size() == 1 );
                os << atomic_name_vec_get( str_index[0] );
                break;

                case graph::print_graph_op:
                CPPAD_ASSERT_UNKNOWN( str_index.size() == 2 );
                os << print_text_vec_get( str_index[0] ) << ",";
                os << print_text_vec_get( str_index[1] );
                break;

                default:
                CPPAD_ASSERT_UNKNOWN( str_index.size() == 0 );
                break;
            }
            os << "\n";
            node_index += n_result;
        }
        //
        //  dependent vector
        size_t n_dependent = dependent_vec_.size();
        os << "y nodes = ";
        for(size_t i = 0; i < n_dependent; i++)
        {   os << dependent_vec_[i];
            if( i + 1 < n_dependent )
                os << ", ";
        }
        os << "\n";
    }

}; // END CPP_GRAPH_CLASS

} // END_CPPAD_NAMESPACE

# endif
