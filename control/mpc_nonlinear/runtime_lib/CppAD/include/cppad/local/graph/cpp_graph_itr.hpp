# ifndef CPPAD_LOCAL_GRAPH_CPP_GRAPH_ITR_HPP
# define CPPAD_LOCAL_GRAPH_CPP_GRAPH_ITR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/utility/vector.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>

// BEGIN_CPPAD_LOCAL_GRAPH_NAMESPACE
namespace CppAD { namespace local { namespace graph {

class cpp_graph_itr {
/*
$begin cpp_graph_itr_data$$
$spell
    Iterator
$$

$section C++ AD Graph Iterator Private Member Data$$
$srccode%hpp% */
private:
    // valuse set by constructor
    const vector<graph_op_enum>*   operator_vec_;
    const vector<size_t>*          operator_arg_;
    //
    // set by constructor and ++
    size_t                         op_index_;
    size_t                         first_arg_;
    //
    // set by get_value
    size_t                         first_node_;
    graph_op_enum                  op_enum_;
    vector<size_t>                 str_index_;
    size_t                         n_result_;
    vector<size_t>                 arg_node_;
/* %$$
$end
------------------------------------------------------------------------------
$begin cpp_graph_itr_get_value$$
$spell
    obj
    op
    arg
    vec
    enum
    Iterator
    itr
    str
$$

$section C++ AD Graph Iterator get_value()$$

$head Syntax$$
$icode%itr%.get_value()%$$

$head op_index_$$
This input is the operator index for the value we are retrieving.

$head first_arg_$$
This input is the first argument index for the value we are retrieving.

$head first_node_$$
The input value of this argument does not matter.
It is set to the index in $code operator_arg_$$
of the first node argument for this operator.

$head op_enum_$$
The input value of this argument does not matter.
It is set to the $cref graph_op_enum$$ for the operator

$head str_index_$$
The input value of this argument does not matter.
Upon return its size is zero except for the special cases
listed below:

$subhead atom_graph_op$$
If $icode op_enum_$$ is $code atom_graph_op$$,
$code str_index_.size() == 1$$ and
$code str_index_[0]$$ is the index in
$cref/atomic_name_vec/cpp_ad_graph/atomic_name_vec/$$
for the function called by this operator.

$subhead discrete_graph_op$$
If $icode op_enum_$$ is $code discrete_graph_op$$,
$code str_index_.size() == 1$$ and
$code str_index_[0]$$ is the index in
$cref/discrete_name_vec/cpp_ad_graph/discrete_name_vec/$$
for the function called by this operator.

$subhead print_graph_op$$
If $icode op_enum_$$ is $code print_graph_op$$,
$code str_index_.size() == 2$$ and
$code str_index_[0]$$ ( $code str_index_[1]$$ )
is the index in
$cref/print_text_vec/cpp_ad_graph/print_text_vec/$$ for the
$cref/before/PrintFor/before/$$  ($cref/after/PrintFor/after/$$) text.

$head n_result_$$
The input value of this argument does not matter.
This is set to the number of result nodes for this operator.

$head arg_node_$$
The input value of this argument does not matter.
Upon return, its size is the number of arguments,
that are node indices, for this operator usage.
The value of the elements are the node indices.

$head Prototype$$
$srccode%hpp% */
    void get_value(void)
/* %$$
$end
*/
{   // initialize output values
    size_t invalid_index   = std::numeric_limits<size_t>::max();
    size_t n_arg      = invalid_index;
    first_node_       = invalid_index;
    n_result_         = invalid_index;
    str_index_.resize(0);
    arg_node_.resize(0);
    //
    // op_enum
    op_enum_          = (*operator_vec_)[op_index_];
    //
    // n_result_, n_arg, str_index_
    switch( op_enum_ )
    {
        // unary operators
        case abs_graph_op:
        case acos_graph_op:
        case acosh_graph_op:
        case asin_graph_op:
        case asinh_graph_op:
        case atan_graph_op:
        case atanh_graph_op:
        case cos_graph_op:
        case cosh_graph_op:
        case erf_graph_op:
        case erfc_graph_op:
        case exp_graph_op:
        case expm1_graph_op:
        case log1p_graph_op:
        case log_graph_op:
        case neg_graph_op:
        case sign_graph_op:
        case sin_graph_op:
        case sinh_graph_op:
        case sqrt_graph_op:
        case tan_graph_op:
        case tanh_graph_op:
        first_node_ = first_arg_;
        n_result_   = 1;
        n_arg       = 1;
        break;

        // binary operators
        case add_graph_op:
        case azmul_graph_op:
        case div_graph_op:
        case mul_graph_op:
        case pow_graph_op:
        case sub_graph_op:
        first_node_ = first_arg_;
        n_result_   = 1;
        n_arg       = 2;
        break;

        // discrete_graph_op
        case discrete_graph_op:
        first_node_ = first_arg_ + 1;
        str_index_.push_back( (*operator_arg_)[first_node_ - 1] );
        n_result_   = 1;
        n_arg       = 1;
        break;


        // atom_graph_op
        case atom_graph_op:
        first_node_ = first_arg_ + 3;
        str_index_.push_back( (*operator_arg_)[first_node_ - 3] );
        n_result_   = (*operator_arg_)[first_node_ - 2];
        n_arg       = (*operator_arg_)[first_node_ - 1];
        break;

        // print_graph_op
        case print_graph_op:
        first_node_ = first_arg_ + 2;
        str_index_.push_back( (*operator_arg_)[first_node_ - 2] );
        str_index_.push_back( (*operator_arg_)[first_node_ - 1] );
        n_result_   = 0;
        n_arg       = 2;
        break;


        // conditional expressions
        case cexp_eq_graph_op:
        case cexp_le_graph_op:
        case cexp_lt_graph_op:
        first_node_ = first_arg_;
        n_result_   = 1;
        n_arg       = 4;
        break;

        // comparison operators
        case comp_eq_graph_op:
        case comp_le_graph_op:
        case comp_lt_graph_op:
        case comp_ne_graph_op:
        first_node_ = first_arg_;
        n_result_   = 0;
        n_arg       = 2;
        break;

        // sum_graph_op
        case sum_graph_op:
        first_node_ = first_arg_ + 1;
        n_result_   = 1;
        n_arg       = (*operator_arg_)[first_node_ - 1];
        break;

        default:
        CPPAD_ASSERT_UNKNOWN(false);
        break;
    }
    // set arg_node
    arg_node_.resize(n_arg);
    for(size_t i = 0; i < n_arg; i++)
        arg_node_[i] = (*operator_arg_)[first_node_ + i];

    return;
}
/* %$$
$end
-------------------------------------------------------------------------------
$begin cpp_graph_itr_types$$
$spell
    Iterator
$$

$section C++ AD Graph Iterator Types$$

$srccode%hpp% */
public:
    typedef struct {
        graph_op_enum          op_enum;
        const vector<size_t>*  str_index_ptr;
        size_t                 n_result;
        const vector<size_t>*  arg_node_ptr;
    } value_type;
    typedef std::input_iterator_tag    iterator_category;
/* %$$
$end
------------------------------------------------------------------------------
$begin cpp_graph_itr_ctor$$
$spell
    Iterator
    itr
    vec
    arg
    op
    cpp
$$

$section C++ AD Graph Iterator Constructors$$

$head Syntax$$
$codei%cpp_graph_itr %default%
%$$
$codei%cpp_graph_itr %itr%(%operator_vec%, %operator_arg%, %op_index%
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_CTOR%// END_CTOR%1
%$$

$head default$$
The result of the default constructor can only be used as a target
for the assignment operator.

$head operator_vec$$
Is the $cref/operator_vec/cpp_ad_graph/operator_vec/$$
for the $code cpp_graph$$ container that this iterator refers to.

$head operator_arg$$
Is the $cref/operator_arg/cpp_ad_graph/operator_vec/$$
for the $code cpp_graph$$ container that this iterator refers to.

$head op_index$$
This must be either zero (the $code begin()$$ for the container)
or equal to the size of $icode operator_vec$$
(the $code end()$$ for the container).

$end
*/
    cpp_graph_itr(void)
    : operator_vec_(nullptr), operator_arg_(nullptr)
    { }
    // BEGIN_CTOR
    cpp_graph_itr(
        const vector<graph_op_enum>& operator_vec   ,
        const vector<size_t>&        operator_arg   ,
        size_t                       op_index       )
    // END_CTOR
    :
    operator_vec_(&operator_vec) ,
    operator_arg_(&operator_arg) ,
    op_index_(op_index)
    {   // end constructor
        if( op_index == operator_vec.size() )
            return;
        //
        // begin constructor
        CPPAD_ASSERT_KNOWN( op_index == 0,
            "cpp_graph_itr: constructor op_index not 0 or operator_vec.size()"
        );
        // start at the beginning of operator_vec
        first_arg_ = 0;
        //
        // get the value, and first_node_, for this operator
        get_value();
    }
/* %$$
------------------------------------------------------------------------------
$begin cpp_graph_itr_input$$
$spell
    Iterator
$$

$section C++ AD Graph Iterator Input Operations$$

$srccode%hpp% */
    // itr == other
    bool operator==(const cpp_graph_itr& other) const
    {   return op_index_ == other.op_index_;
    }
    // itr != other
    bool operator!=(const cpp_graph_itr& other) const
    {   return op_index_ != other.op_index_;
    }
    // *itr
    value_type operator*(void)
    {   CPPAD_ASSERT_KNOWN( operator_vec_ != nullptr,
            "cpp_graph_itr: attempt to dereference default iterator"
        );
        CPPAD_ASSERT_KNOWN( op_index_ < operator_vec_->size(),
            "cpp_graph_itr: attempt to dereference past last element in graph"
        );
        value_type ret;
        ret.op_enum       = op_enum_;
        ret.str_index_ptr = &str_index_;
        ret.n_result      = n_result_;
        ret.arg_node_ptr  = &arg_node_;
        return ret;
    }
    // ++itr
    cpp_graph_itr& operator++(void)
    {   ++op_index_;
        first_arg_ = first_node_ + arg_node_.size();
        get_value();
        return *this;
    }
    // itr++
    cpp_graph_itr operator++(int)
    {   cpp_graph_itr ret(*this);
        ++op_index_;
        first_arg_ = first_node_ + arg_node_.size();
        get_value();
        return ret;
    }
/* %$$
$end
*/

};

} } } // END_CPPAD_LOCAL_GRAPH_NAMESPACE

# endif
