# ifndef CPPAD_CORE_AD_FUN_HPP
# define CPPAD_CORE_AD_FUN_HPP
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
$begin ADFun$$
$spell
    xk
    Ind
    bool
    taylor_
    sizeof
    const
    std
    ind_taddr_
    dep_taddr_
$$

$spell
$$

$section ADFun Objects$$


$head Purpose$$
An AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$
is stored in an $code ADFun$$ object by its $cref FunConstruct$$.
The $code ADFun$$ object can then be used to calculate function values,
derivative values, and other values related to the corresponding function.

$childtable%
    include/cppad/core/ad_fun.omh%
    include/cppad/core/optimize.hpp%
    include/cppad/core/fun_check.hpp%
    include/cppad/core/check_for_nan.hpp
%$$

$end
*/
# include <cppad/core/graph/cpp_graph.hpp>
# include <cppad/local/subgraph/info.hpp>
# include <cppad/local/graph/cpp_graph_op.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file ad_fun.hpp
File used to define the ADFun<Base> class.
*/

/*!
Class used to hold function objects

\tparam Base
A function object has a recording of <tt>AD<Base></tt> operations.
It does it calculations using Base operations.
*/


template <class Base, class RecBase>
class ADFun {
    // ADFun<Base> must be a friend of ADFun< AD<Base> > for base2ad to work.
    template <class Base2, class RecBase2> friend class ADFun;
private:
    // ------------------------------------------------------------
    // Private member variables
    // ------------------------------------------------------------

    /// name of this function (so far only json operations use this value)
    std::string function_name_;

    /// Did the previous optimzation exceed the collision limit
    bool exceed_collision_limit_;

    /// Has this ADFun object been optmized
    bool has_been_optimized_;

    /// Check for nan's and report message to user (default value is true).
    bool check_for_nan_;

    /// If zero, ignoring comparison operators. Otherwise is the
    /// compare change count at which to store the operator index.
    size_t compare_change_count_;

    /// If compare_change_count_ is zero, compare_change_number_ is also zero.
    /// Otherwise, it is set to the number of comparison operations that had a
    /// different result during the subsequent zero order forward.
    size_t compare_change_number_;

    /// If compare_change_count is zero, compare_change_op_index_ is also
    /// zero. Otherwise it is the operator index for the comparison operator
    //// that corresponded to the number changing from count-1 to count.
    size_t compare_change_op_index_;

    /// number of orders stored in taylor_
    size_t num_order_taylor_;

    /// maximum number of orders that will fit in taylor_
    size_t cap_order_taylor_;

    /// number of directions stored in taylor_
    size_t num_direction_taylor_;

    /// number of variables in the recording (play_)
    size_t num_var_tape_;

    /// tape address for the independent variables
    local::pod_vector<size_t> ind_taddr_;

    /// tape address and parameter flag for the dependent variables
    local::pod_vector<size_t> dep_taddr_;

    /// which dependent variables are actually parameters
    local::pod_vector<bool> dep_parameter_;

    /// which operations can be conditionally skipped
    /// Set during forward pass of order zero
    local::pod_vector<bool> cskip_op_;

    /// Variable on the tape corresponding to each vecad load operation
    /// (if zero, the operation corresponds to a parameter).
    local::pod_vector<addr_t> load_op2var_;

    /// results of the forward mode calculations
    local::pod_vector_maybe<Base> taylor_;

    /// used for subgraph reverse mode calculations.
    /// Declared here to avoid reallocation for each call to subgraph_reverse.
    /// Not in subgraph_info_ because it depends on Base.
    local::pod_vector_maybe<Base> subgraph_partial_;

    /// the operation sequence corresponding to this object
    local::player<Base> play_;

    /// subgraph information for this object
    local::subgraph::subgraph_info subgraph_info_;

    /// Packed results of the forward mode Jacobian sparsity calculations.
    /// for_jac_sparse_pack_.n_set() != 0  implies other sparsity results
    /// are empty
    local::sparse::pack_setvec for_jac_sparse_pack_;

    /// Set results of the forward mode Jacobian sparsity calculations
    /// for_jac_sparse_set_.n_set() != 0  implies for_sparse_pack_ is empty.
    local::sparse::list_setvec for_jac_sparse_set_;


    // ------------------------------------------------------------
    // Private member functions
    // ------------------------------------------------------------

    /// change the operation sequence corresponding to this object
    template <class ADvector>
    void Dependent(local::ADTape<Base> *tape, const ADvector &y);

    // vector of bool version of ForSparseJac
    // (doxygen in cppad/core/for_sparse_jac.hpp)
    template <class SetVector>
    void ForSparseJacCase(
        bool               set_type  ,
        bool               transpose ,
        bool               dependency,
        size_t             q         ,
        const SetVector&   r         ,
        SetVector&         s
    );

    // vector of std::set<size_t> version of ForSparseJac
    // (doxygen in cppad/core/for_sparse_jac.hpp)
    template <class SetVector>
    void ForSparseJacCase(
        const std::set<size_t>&  set_type  ,
        bool                     transpose ,
        bool                     dependency,
        size_t                   q         ,
        const SetVector&         r         ,
        SetVector&               s
    );

    // vector of bool version of RevSparseJac
    // (doxygen in cppad/core/rev_sparse_jac.hpp)
    template <class SetVector>
    void RevSparseJacCase(
        bool               set_type  ,
        bool               transpose ,
        bool               dependency,
        size_t             p         ,
        const SetVector&   s         ,
        SetVector&         r
    );

    // vector of std::set<size_t> version of RevSparseJac
    // (doxygen in cppad/core/rev_sparse_jac.hpp)
    template <class SetVector>
    void RevSparseJacCase(
        const std::set<size_t>&  set_type  ,
        bool                     transpose ,
        bool                     dependency,
        size_t                   p         ,
        const SetVector&         s         ,
        SetVector&               r
    );

    // vector of bool version of ForSparseHes
    // (doxygen in cppad/core/for_sparse_hes.hpp)
    template <class SetVector>
    void ForSparseHesCase(
        bool               set_type  ,
        const SetVector&   r         ,
        const SetVector&   s         ,
        SetVector&         h
    );

    // vector of std::set<size_t> version of ForSparseHes
    // (doxygen in cppad/core/for_sparse_hes.hpp)
    template <class SetVector>
    void ForSparseHesCase(
        const std::set<size_t>&  set_type  ,
        const SetVector&         r         ,
        const SetVector&         s         ,
        SetVector&               h
    );

    // vector of bool version of RevSparseHes
    // (doxygen in cppad/core/rev_sparse_hes.hpp)
    template <class SetVector>
    void RevSparseHesCase(
        bool               set_type  ,
        bool               transpose ,
        size_t             q         ,
        const SetVector&   s         ,
        SetVector&         h
    );

    // vector of std::set<size_t> version of RevSparseHes
    // (doxygen in cppad/core/rev_sparse_hes.hpp)
    template <class SetVector>
    void RevSparseHesCase(
        const std::set<size_t>&  set_type  ,
        bool                     transpose ,
        size_t                   q         ,
        const SetVector&         s         ,
        SetVector&               h
    );

    // Forward mode version of SparseJacobian
    // (doxygen in cppad/core/sparse_jacobian.hpp)
    template <class BaseVector, class SetVector, class SizeVector>
    size_t SparseJacobianFor(
        const BaseVector&           x               ,
              SetVector&            p_transpose     ,
        const SizeVector&           row             ,
        const SizeVector&           col             ,
              BaseVector&           jac             ,
              sparse_jacobian_work& work
    );

    // Reverse mode version of SparseJacobian
    // (doxygen in cppad/core/sparse_jacobian.hpp)
    template <class BaseVector, class SetVector, class SizeVector>
    size_t SparseJacobianRev(
        const BaseVector&           x               ,
              SetVector&            p               ,
        const SizeVector&           row             ,
        const SizeVector&           col             ,
              BaseVector&           jac             ,
              sparse_jacobian_work& work
    );

    // combined sparse_list and sparse_pack version of SparseHessian
    // (doxygen in cppad/core/sparse_hessian.hpp)
    template <class BaseVector, class SetVector, class SizeVector>
    size_t SparseHessianCompute(
        const BaseVector&              x           ,
        const BaseVector&              w           ,
              SetVector&               sparsity    ,
        const SizeVector&              row         ,
        const SizeVector&              col         ,
              BaseVector&              hes         ,
              sparse_hessian_work&     work
    );

public:
    /// default constructor
    ADFun(void);

    /// copy constructor
    ADFun(const ADFun& g) = delete;

    // assignment operator
    // (doxygen in cppad/core/fun_construct.hpp)
    void operator=(const ADFun& f);

    // swap
    void swap(ADFun& f);

    // move semenatics copy
    ADFun(ADFun&& f);

    // move semantics assignment
    void operator=(ADFun&& f);

    // create from Json or C++ AD graph
    void from_json(const std::string& json);
    void from_graph(const cpp_graph& graph_obj);
    void from_graph(
        const cpp_graph&    graph_obj  ,
        const vector<bool>& dyn2var    ,
        const vector<bool>& var2dyn
    );

    // create a Json or C++ AD graph
    std::string to_json(void);
    void to_graph(cpp_graph& graph_obj);

    // create ADFun< AD<Base> > from this ADFun<Base>
    // (doxygen in cppad/core/base2ad.hpp)
    ADFun< AD<Base>, RecBase > base2ad(void) const;

    /// sequence constructor
    template <class ADvector>
    ADFun(const ADvector &x, const ADvector &y);

    /// destructor
    ~ADFun(void);

    /// set check_for_nan
    void check_for_nan(bool value);

    /// get check_for_nan
    bool check_for_nan(void) const;

    /// assign a new operation sequence
    template <class ADvector>
    void Dependent(const ADvector &x, const ADvector &y);

    /// new_dynamic user API
    template <class BaseVector>
    void new_dynamic(const BaseVector& dynamic);

    /// forward mode user API, one order multiple directions.
    template <class BaseVector>
    BaseVector Forward(size_t q, size_t r, const BaseVector& x);

    /// forward mode user API, multiple orders one direction.
    template <class BaseVector>
    BaseVector Forward(
        size_t q, const BaseVector& xq, std::ostream& s = std::cout
    );

    /// reverse mode sweep
    template <class BaseVector>
    BaseVector Reverse(size_t p, const BaseVector &v);

    // forward Jacobian sparsity pattern
    // (doxygen in cppad/core/for_sparse_jac.hpp)
    template <class SetVector>
    SetVector ForSparseJac(
        size_t q, const SetVector &r, bool transpose = false,
        bool dependency = false
    );

    // reverse Jacobian sparsity pattern
    // (doxygen in cppad/core/rev_sparse_jac.hpp)
    template <class SetVector>
    SetVector RevSparseJac(
        size_t q, const SetVector &s, bool transpose = false,
        bool dependency = false
    );

    // subgraph_reverse: select domain
    // (doxygen in cppad/core/subgraph_reverse.hpp)
    template <class BoolVector>
    void subgraph_reverse(
        const BoolVector&                   select_domain
    );

    // subgraph_reverse: compute derivative
    // (doxygen in cppad/core/subgraph_reverse.hpp)
    template <class Addr, class BaseVector, class SizeVector>
    void subgraph_reverse_helper(
        size_t                               q         ,
        size_t                               ell       ,
        SizeVector&                          col       ,
        BaseVector&                          dw
    );

    // subgraph_reverse: compute derivative
    // (doxygen in cppad/core/subgraph_reverse.hpp)
    template <class BaseVector, class SizeVector>
    void subgraph_reverse(
        size_t                               q         ,
        size_t                               ell       ,
        SizeVector&                          col       ,
        BaseVector&                          dw
    );

    // subgraph_jac_rev: compute Jacobian
    // (doxygen in cppad/core/subgraph_jac_rev.hpp)
    template <class SizeVector, class BaseVector>
    void subgraph_jac_rev(
        const BaseVector&                    x         ,
        sparse_rcv<SizeVector, BaseVector>&  subset
    );

    // subgraph_jac_rev: compute Jacobian
    // (doxygen missing in cppad/core/subgraph_jac_rev.hpp)
    template <class BoolVector, class SizeVector, class BaseVector>
    void subgraph_jac_rev(
        const BoolVector&                    select_domain ,
        const BoolVector&                    select_range  ,
        const BaseVector&                    x             ,
        sparse_rcv<SizeVector, BaseVector>&  matrix_out
    );


    // compute sparse Jacobian using forward mode
    // (doxygen in cppad/core/sparse_jac.hpp)
    template <class SizeVector, class BaseVector>
    size_t sparse_jac_for(
        size_t                               group_max ,
        const BaseVector&                    x         ,
        sparse_rcv<SizeVector, BaseVector>&  subset    ,
        const sparse_rc<SizeVector>&         pattern   ,
        const std::string&                   coloring  ,
        sparse_jac_work&                     work
    );

    // compute sparse Jacobian using reverse mode
    // (doxygen in cppad/core/sparse_jac.hpp)
    template <class SizeVector, class BaseVector>
    size_t sparse_jac_rev(
        const BaseVector&                    x        ,
        sparse_rcv<SizeVector, BaseVector>&  subset   ,
        const sparse_rc<SizeVector>&         pattern  ,
        const std::string&                   coloring ,
        sparse_jac_work&                     work
    );

    // compute sparse Hessian
    // (doxygen in cppad/core/sparse_hes.hpp)
    template <class SizeVector, class BaseVector>
    size_t sparse_hes(
        const BaseVector&                    x        ,
        const BaseVector&                    w        ,
        sparse_rcv<SizeVector, BaseVector>&  subset   ,
        const sparse_rc<SizeVector>&         pattern  ,
        const std::string&                   coloring ,
        sparse_hes_work&                     work
    );

    // compute sparsity pattern using subgraphs
    // (doxygen in cppad/core/subgraph_sparsity.hpp)
    template <class BoolVector, class SizeVector>
    void subgraph_sparsity(
        const BoolVector&            select_domain    ,
        const BoolVector&            select_range     ,
        bool                         transpose        ,
        sparse_rc<SizeVector>&       pattern_out
    );


    // forward mode Jacobian sparsity pattern
    // (doxygen in cppad/core/for_jac_sparsity.hpp)
    template <class SizeVector>
    void for_jac_sparsity(
        const sparse_rc<SizeVector>& pattern_in       ,
        bool                         transpose        ,
        bool                         dependency       ,
        bool                         internal_bool    ,
        sparse_rc<SizeVector>&       pattern_out
    );

    // reverse mode Jacobian sparsity pattern
    // (doxygen in cppad/core/for_jac_sparsity.hpp)
    template <class SizeVector>
    void rev_jac_sparsity(
        const sparse_rc<SizeVector>& pattern_in       ,
        bool                         transpose        ,
        bool                         dependency       ,
        bool                         internal_bool    ,
        sparse_rc<SizeVector>&       pattern_out
    );

    // reverse mode Hessian sparsity pattern
    // (doxygen in cppad/core/rev_hes_sparsity.hpp)
    template <class BoolVector, class SizeVector>
    void rev_hes_sparsity(
        const BoolVector&            select_range     ,
        bool                         transpose        ,
        bool                         internal_bool    ,
        sparse_rc<SizeVector>&       pattern_out
    );

    // forward mode Hessian sparsity pattern
    // (doxygen in cppad/core/for_hes_sparsity.hpp)
    template <class BoolVector, class SizeVector>
    void for_hes_sparsity(
        const BoolVector&            select_domain    ,
        const BoolVector&            select_range     ,
        bool                         internal_bool    ,
        sparse_rc<SizeVector>&       pattern_out
    );

    // forward mode Hessian sparsity pattern
    // (see doxygen in cppad/core/for_sparse_hes.hpp)
    template <class SetVector>
    SetVector ForSparseHes(
        const SetVector &r, const SetVector &s
    );

    // internal set sparsity version of ForSparseHes
    // (used by checkpoint functions only)
    void ForSparseHesCheckpoint(
        vector<bool>&                 r         ,
        vector<bool>&                 s         ,
        local::sparse::list_setvec&   h
    );

    // reverse mode Hessian sparsity pattern
    // (see doxygen in cppad/core/rev_sparse_hes.hpp)
    template <class SetVector>
    SetVector RevSparseHes(
        size_t q, const SetVector &s, bool transpose = false
    );

    // internal set sparsity version of RevSparseHes
    // (doxygen in cppad/core/rev_sparse_hes.hpp)
    // (used by checkpoint functions only)
    void RevSparseHesCheckpoint(
        size_t                        q         ,
        vector<bool>&                 s         ,
        bool                          transpose ,
        local::sparse::list_setvec&   h
    );

    // internal set sparsity version of RevSparseJac
    // (doxygen in cppad/core/rev_sparse_jac.hpp)
    // (used by checkpoint functions only)
    void RevSparseJacCheckpoint(
        size_t                        q          ,
        const local::sparse::list_setvec&     r          ,
        bool                          transpose  ,
        bool                          dependency ,
        local::sparse::list_setvec&   s
    );

    // internal set sparsity version of RevSparseJac
    // (doxygen in cppad/core/for_sparse_jac.hpp)
    // (used by checkpoint functions only)
    void ForSparseJacCheckpoint(
    size_t                             q          ,
    const local::sparse::list_setvec&  r          ,
    bool                               transpose  ,
    bool                               dependency ,
    local::sparse::list_setvec&        s
    );

    /// did previous optimization exceed the collision limit
    bool exceed_collision_limit(void) const
    {   return exceed_collision_limit_; }

    /// amount of memory used for boolean Jacobain sparsity pattern
    size_t size_forward_bool(void) const
    {   return for_jac_sparse_pack_.memory(); }

    /// free memory used for Jacobain sparsity pattern
    void size_forward_bool(size_t zero)
    {   CPPAD_ASSERT_KNOWN(
            zero == 0,
            "size_forward_bool: argument not equal to zero"
        );
        for_jac_sparse_pack_.resize(0, 0);
    }

    /// amount of memory used for vector of set Jacobain sparsity pattern
    size_t size_forward_set(void) const
    {   return for_jac_sparse_set_.memory(); }

    /// free memory used for Jacobain sparsity pattern
    void size_forward_set(size_t zero)
    {   CPPAD_ASSERT_KNOWN(
            zero == 0,
            "size_forward_bool: argument not equal to zero"
        );
        for_jac_sparse_set_.resize(0, 0);
    }

    /// number of operators in the operation sequence
    size_t size_op(void) const
    {   return play_.num_op_rec(); }

    /// number of operator arguments in the operation sequence
    size_t size_op_arg(void) const
    {   return play_.num_op_arg_rec(); }

    /// amount of memory required for the operation sequence
    size_t size_op_seq(void) const
    {   return play_.size_op_seq(); }

    /// amount of memory currently allocated for random access
    /// of the operation sequence
    size_t size_random(void) const
    {   return play_.size_random(); }

    /// number of parameters in the operation sequence
    size_t size_par(void) const
    {   return play_.num_par_rec(); }

    /// number of independent dynamic parameters
    size_t size_dyn_ind(void) const
    {   return play_.num_dynamic_ind(); }

    /// number of dynamic parameters
    size_t size_dyn_par(void) const
    {   return play_.num_dynamic_par(); }

    /// number of dynamic parameters arguments
    size_t size_dyn_arg(void) const
    {   return play_.num_dynamic_arg(); }

    /// number taylor coefficient orders calculated
    size_t size_order(void) const
    {   return num_order_taylor_; }

    /// number taylor coefficient directions calculated
    size_t size_direction(void) const
    {   return num_direction_taylor_; }

    /// number of characters in the operation sequence
    size_t size_text(void) const
    {   return play_.num_text_rec(); }

    /// number of variables in opertion sequence
    size_t size_var(void) const
    {   return num_var_tape_; }

    /// number of VecAD indices in the operation sequence
    size_t size_VecAD(void) const
    {   return play_.num_var_vecad_ind_rec(); }

    /// set number of orders currently allocated (user API)
    void capacity_order(size_t c);

    /// set number of orders and directions currently allocated
    void capacity_order(size_t c, size_t r);

    /// number of variables in conditional expressions that can be skipped
    size_t number_skip(void);

    /// number of independent variables
    size_t Domain(void) const
    {   return ind_taddr_.size(); }

    /// number of dependent variables
    size_t Range(void) const
    {   return dep_taddr_.size(); }

    /// set and get function name
    void function_name_set(const std::string& function_name)
    {   function_name_ = function_name; }
    std::string function_name_get(void)
    {   return function_name_; }

    /// is variable a parameter
    bool Parameter(size_t i)
    {   CPPAD_ASSERT_KNOWN(
            i < dep_taddr_.size(),
            "Argument to Parameter is >= dimension of range space"
        );
        return dep_parameter_[i];
    }

    /// Deprecated: number of comparison operations that changed
    /// for the previous zero order forward (than when function was recorded)
    size_t CompareChange(void) const
    {   return compare_change_number_; }

    /// count as which to store operator index
    void compare_change_count(size_t count)
    {   compare_change_count_    = count;
        compare_change_number_   = 0;
        compare_change_op_index_ = 0;
    }

    /// number of comparison operations that changed
    size_t compare_change_number(void) const
    {   return compare_change_number_; }

    /// operator index for the count-th  comparison change
    size_t compare_change_op_index(void) const
    {   if( has_been_optimized_ )
            return 0;
        return compare_change_op_index_;
    }

    /// calculate entire Jacobian
    template <class BaseVector>
    BaseVector Jacobian(const BaseVector &x);

    /// calculate Hessian for one component of f
    template <class BaseVector>
    BaseVector Hessian(const BaseVector &x, const BaseVector &w);
    template <class BaseVector>
    BaseVector Hessian(const BaseVector &x, size_t i);

    /// forward mode calculation of partial w.r.t one domain component
    template <class BaseVector>
    BaseVector ForOne(
        const BaseVector   &x ,
        size_t              j );

    /// reverse mode calculation of derivative of one range component
    template <class BaseVector>
    BaseVector RevOne(
        const BaseVector   &x ,
        size_t              i );

    /// forward mode calculation of a subset of second order partials
    template <class BaseVector, class SizeVector_t>
    BaseVector ForTwo(
        const BaseVector   &x ,
        const SizeVector_t &J ,
        const SizeVector_t &K );

    /// reverse mode calculation of a subset of second order partials
    template <class BaseVector, class SizeVector_t>
    BaseVector RevTwo(
        const BaseVector   &x ,
        const SizeVector_t &I ,
        const SizeVector_t &J );

    /// calculate sparse Jacobians
    template <class BaseVector>
    BaseVector SparseJacobian(
        const BaseVector &x
    );
    template <class BaseVector, class SetVector>
    BaseVector SparseJacobian(
        const BaseVector &x ,
        const SetVector  &p
    );
    template <class BaseVector, class SetVector, class SizeVector>
    size_t SparseJacobianForward(
        const BaseVector&     x     ,
        const SetVector&      p     ,
        const SizeVector&     r     ,
        const SizeVector&     c     ,
        BaseVector&           jac   ,
        sparse_jacobian_work& work
    );
    template <class BaseVector, class SetVector, class SizeVector>
    size_t SparseJacobianReverse(
        const BaseVector&     x    ,
        const SetVector&      p    ,
        const SizeVector&     r    ,
        const SizeVector&     c    ,
        BaseVector&           jac  ,
        sparse_jacobian_work& work
    );

    /// calculate sparse Hessians
    template <class BaseVector>
    BaseVector SparseHessian(
        const BaseVector&    x  ,
        const BaseVector&    w
    );
    template <class BaseVector, class BoolVector>
    BaseVector SparseHessian(
        const BaseVector&    x  ,
        const BaseVector&    w  ,
        const BoolVector&    p
    );
    template <class BaseVector, class SetVector, class SizeVector>
    size_t SparseHessian(
        const BaseVector&    x   ,
        const BaseVector&    w   ,
        const SetVector&     p   ,
        const SizeVector&    r   ,
        const SizeVector&    c   ,
        BaseVector&          hes ,
        sparse_hessian_work& work
    );

    // Optimize the tape
    // (see doxygen documentation in optimize.hpp)
    void optimize( const std::string& options = "" );

    // create abs-normal representation of the function f(x)
    void abs_normal_fun( ADFun& g, ADFun& a ) const;

    // clear all subgraph information
    void clear_subgraph(void);
    // ------------------- Deprecated -----------------------------

    /// deprecated: assign a new operation sequence
    template <class ADvector>
    void Dependent(const ADvector &y);

    /// Deprecated: number of variables in opertion sequence
    size_t Size(void) const
    {   return num_var_tape_; }

    /// Deprecated: # taylor_ coefficients currently stored
    /// (per variable,direction)
    size_t Order(void) const
    {   return num_order_taylor_ - 1; }

    /// Deprecated: amount of memory for this object
    /// Note that an approximation is used for the std::set<size_t> memory
    size_t Memory(void) const
    {   size_t pervar  = cap_order_taylor_ * sizeof(Base)
        + for_jac_sparse_pack_.memory()
        + for_jac_sparse_set_.memory();
        size_t total   = num_var_tape_  * pervar;
        total         += play_.size_op_seq();
        total         += play_.size_random();
        total         += subgraph_info_.memory();
        return total;
    }

    /// Deprecated: # taylor_ coefficient orderss stored
    /// (per variable,direction)
    size_t taylor_size(void) const
    {   return num_order_taylor_; }

    /// Deprecated: Does this AD operation sequence use
    /// VecAD<Base>::reference operands
    bool use_VecAD(void) const
    {   return play_.num_var_vecad_ind_rec() > 0; }

    /// Deprecated: # taylor_ coefficient orders calculated
    /// (per variable,direction)
    size_t size_taylor(void) const
    {   return num_order_taylor_; }

    /// Deprecated: set number of orders currently allocated
    /// (per variable,direction)
    void capacity_taylor(size_t per_var);
};
// ---------------------------------------------------------------------------

} // END_CPPAD_NAMESPACE

// non-user interfaces
# include <cppad/local/sweep/forward0.hpp>
# include <cppad/local/sweep/forward1.hpp>
# include <cppad/local/sweep/forward2.hpp>
# include <cppad/local/sweep/reverse.hpp>
# include <cppad/local/sweep/for_jac.hpp>
# include <cppad/local/sweep/rev_jac.hpp>
# include <cppad/local/sweep/rev_hes.hpp>
# include <cppad/local/sweep/for_hes.hpp>
# include <cppad/core/graph/from_graph.hpp>
# include <cppad/core/graph/to_graph.hpp>

// user interfaces
# include <cppad/core/parallel_ad.hpp>
# include <cppad/core/independent/independent.hpp>
# include <cppad/core/dependent.hpp>
# include <cppad/core/fun_construct.hpp>
# include <cppad/core/base2ad.hpp>
# include <cppad/core/abort_recording.hpp>
# include <cppad/core/fun_eval.hpp>
# include <cppad/core/drivers.hpp>
# include <cppad/core/fun_check.hpp>
# include <cppad/core/omp_max_thread.hpp>
# include <cppad/core/optimize.hpp>
# include <cppad/core/abs_normal_fun.hpp>
# include <cppad/core/graph/from_json.hpp>
# include <cppad/core/graph/to_json.hpp>

# endif
