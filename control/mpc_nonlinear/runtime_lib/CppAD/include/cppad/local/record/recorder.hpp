# ifndef CPPAD_LOCAL_RECORD_RECORDER_HPP
# define CPPAD_LOCAL_RECORD_RECORDER_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/core/hash_code.hpp>
# include <cppad/local/pod_vector.hpp>
# include <cppad/core/ad_type.hpp>

// ----------------------------------------------------------------------------
namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file recorder.hpp
File used to define the recorder class.
*/

/*!
Class used to store an operation sequence while it is being recorded
(the operation sequence is copied to the player class for playback).

\tparam Base
This is an AD< Base > operation sequence recording; i.e.,
it records operations of type AD< Base >.
*/
template <class Base>
class recorder {
    friend class player<Base>;

private:
    /// are comparison operators being recorded
    bool record_compare_;

    /// operator index at which to abort recording with an error
    /// (do not abort when zero)
    size_t abort_op_index_;

    /// Number of variables in the recording.
    size_t num_var_rec_;

    /// Number of dynamic parameters in the recording
    size_t num_dynamic_ind_;

    /// Number vecad load operations (LdpOp or LdvOp) currently in recording.
    size_t num_var_load_rec_;

    /// The operators in the recording.
    pod_vector<opcode_t> op_vec_;

    /// The VecAD indices in the recording.
    pod_vector<addr_t> all_dyn_vecad_ind_;
    pod_vector<addr_t> all_var_vecad_ind_;

    /// The argument indices in the recording
    pod_vector<addr_t> arg_vec_;

    /// Character strings ('\\0' terminated) in the recording.
    pod_vector<char> text_vec_;

    /// Hash table to reduced number of duplicate parameters in all_par_vec_
    pod_vector<addr_t> par_hash_table_;

    /// Vector containing all the parameters in the recording.
    /// Use pod_vector_maybe because Base may not be plain old data.
    pod_vector_maybe<Base> all_par_vec_;

    /// Which elements of all_par_vec_ are dynamic parameters
    /// (same size are all_par_vec_)
    pod_vector<bool> dyn_par_is_;

    /// operators for just the dynamic parameters in all_par_vec_
    pod_vector<opcode_t> dyn_par_op_;

    /// arguments for the dynamic parameter operators
    pod_vector<addr_t> dyn_par_arg_;

// ---------------------- Public Functions -----------------------------------
public:
    /// Default constructor
    recorder(void) :
    num_var_rec_(0)                          ,
    num_dynamic_ind_(0)                      ,
    num_var_load_rec_(0)                      ,
    par_hash_table_( CPPAD_HASH_TABLE_SIZE )
    {   record_compare_ = true;
        abort_op_index_ = 0;
        // It does not matter if unitialized hash codes match but this
        // initilaization is here to avoid valgrind warnings.
        void*  ptr   = static_cast<void*>( par_hash_table_.data() );
        int    value = 0;
        size_t num   = CPPAD_HASH_TABLE_SIZE * sizeof(addr_t);
        std::memset(ptr, value, num);
    }

    /// Set record_compare option
    void set_record_compare(bool record_compare)
    {   record_compare_ = record_compare; }

    /// Set the abort index
    void set_abort_op_index(size_t abort_op_index)
    {   abort_op_index_ = abort_op_index; }

    /// Set number of independent dynamic parameters
    void set_num_dynamic_ind(size_t num_dynamic_ind)
    {   num_dynamic_ind_ = num_dynamic_ind; }

    /// Get record_compare option
    bool get_record_compare(void) const
    {   return record_compare_; }

    /// Get the abort_op_index
    size_t get_abort_op_index(void) const
    {   return abort_op_index_; }

    /// Get number of independent dynamic parameters
    size_t get_num_dynamic_ind(void) const
    {   return num_dynamic_ind_; }

    /// Destructor
    ~recorder(void)
    { }

    /// Put a dynamic parameter in all_par_vec_.
    addr_t put_dyn_par(
        const Base &par, op_code_dyn op
    );
    addr_t put_dyn_par(
        const Base &par, op_code_dyn op, addr_t arg0
    );
    addr_t put_dyn_par(
        const Base &par, op_code_dyn op, addr_t arg0, addr_t arg1
    );
    addr_t put_dyn_cond_exp(
        const Base &par, CompareOp cop,
        addr_t left, addr_t right, addr_t if_true, addr_t if_false
    );

    /// Put a vector of dynamic parameter arguments at end of tape
    void put_dyn_arg_vec(const pod_vector<addr_t>& arg);

    /// Put next operator in the operation sequence.
    addr_t PutOp(OpCode op);
    /// Put a vecad load operator in the operation sequence (special case)
    addr_t PutLoadOp(OpCode op);

    // VecAD operations
    addr_t put_var_vecad_ind(addr_t vec_ind);
    addr_t put_var_vecad(size_t length, const pod_vector<addr_t>& taddr);

    /// Find or add a constant parameter to the vector of all parameters.
    addr_t put_con_par(const Base &par);
    /// Put one operation argument index in the recording
    void PutArg(addr_t arg0);
    /// Put two operation argument index in the recording
    void PutArg(addr_t arg0, addr_t arg1);
    /// Put three operation argument index in the recording
    void PutArg(addr_t arg0, addr_t arg1, addr_t arg2);
    /// Put four operation argument index in the recording
    void PutArg(addr_t arg0, addr_t arg1, addr_t arg2, addr_t arg3);
    /// Put five operation argument index in the recording
    void PutArg(addr_t arg0, addr_t arg1, addr_t arg2, addr_t arg3,
        addr_t arg4);
    /// Put six operation argument index in the recording
    void PutArg(addr_t arg0, addr_t arg1, addr_t arg2, addr_t arg3,
        addr_t arg4, addr_t arg5);
    //
    // put_dyn_atomic
    template <class VectorAD>
    void put_dyn_atomic(
        tape_id_t                   tape_id    ,
        size_t                      atom_index ,
        const vector<ad_type_enum>& type_x     ,
        const vector<ad_type_enum>& type_y     ,
        const VectorAD&             ax         ,
        VectorAD&                   ay
    );
    //
    // put_var_atomic
    template <class VectorAD>
    void put_var_atomic(
        tape_id_t                   tape_id    ,
        size_t                      atom_index ,
        const vector<ad_type_enum>& type_x     ,
        const vector<ad_type_enum>& type_y     ,
        const VectorAD&             ax         ,
        VectorAD&                   ay
    );

    // Reserve space for a specified number of arguments
    size_t ReserveArg(size_t n_arg);

    // Replace an argument value
    void ReplaceArg(size_t i_arg, addr_t value);

    /// Put a character string in the text for this recording.
    addr_t PutTxt(const char *text);

    /// record a variable or dynamic parameter conditional expression
    void cond_exp(
        tape_id_t       tape_id     ,
        enum CompareOp  cop         ,
        AD<Base>       &result      ,
        const AD<Base> &left        ,
        const AD<Base> &right       ,
        const AD<Base> &if_true     ,
        const AD<Base> &if_false
    );

    /// record a comparison operators for varialbes or just dynamic parameters
    void comp_eq(
        bool                        var_left     ,
        bool                        var_right    ,
        bool                        dyn_left     ,
        bool                        dyn_right    ,
        const AD<Base>&             aleft        ,
        const AD<Base>&             aright       ,
        bool                        result
    );
    void comp_le(
        bool                        var_left     ,
        bool                        var_right    ,
        bool                        dyn_left     ,
        bool                        dyn_right    ,
        const AD<Base>&             aleft        ,
        const AD<Base>&             aright       ,
        bool                        result
    );
    void comp_lt(
        bool                        var_left     ,
        bool                        var_right    ,
        bool                        dyn_left     ,
        bool                        dyn_right    ,
        const AD<Base>&             aleft        ,
        const AD<Base>&             aright       ,
        bool                        result
    );

    // -----------------------------------------------------------------------
    // functions implemented here

    /// Number of variables currently stored in the recording.
    size_t num_var_rec(void) const
    {   return num_var_rec_; }

    /// Number LdpOp, LdvOp, and load_dyn operations currently in recording
    size_t num_var_load_rec(void) const
    {   return num_var_load_rec_; }

    /// Number of operators currently stored in the recording.
    size_t num_op_rec(void) const
    {   return  op_vec_.size(); }

    /// current parameter vector
    const pod_vector_maybe<Base>& all_par_vec(void) const
    {   return all_par_vec_; }

    /// Approximate amount of memory used by the recording
    size_t Memory(void) const
    {   return op_vec_.capacity()        * sizeof(opcode_t)
             + all_dyn_vecad_ind_.capacity() * sizeof(addr_t)
             + all_var_vecad_ind_.capacity() * sizeof(addr_t)
             + arg_vec_.capacity()       * sizeof(addr_t)
             + all_par_vec_.capacity()   * sizeof(Base)
             + text_vec_.capacity()      * sizeof(char);
    }

};

/*!
Put next operator in the operation sequence.

This sets the op code for the next operation in this recording.
This call must be followed by putting the corresponding
\verbatim
    NumArg(op)
\endverbatim
argument indices in the recording.

\param op
Is the op code corresponding to the the operation that is being
recorded (which must not be LdpOp or LdvOp).

\return
The return value is the index of the primary (last) variable
corresponding to the result of this operation.
The number of variables corresponding to the operation is given by
\verbatim
    NumRes(op)
\endverbatim
With each call to PutOp or PutLoadOp,
the return index increases by the number of variables corresponding
to the call.
This index starts at zero after the default constructor.
*/
template <class Base>
addr_t recorder<Base>::PutOp(OpCode op)
{   size_t i    = op_vec_.extend(1);
    CPPAD_ASSERT_KNOWN(
        (abort_op_index_ == 0) || (abort_op_index_ != i),
        "Operator index equals abort_op_index in Independent"
    );
    op_vec_[i]  = static_cast<opcode_t>(op);
    CPPAD_ASSERT_UNKNOWN( op_vec_.size() == i + 1 );
    CPPAD_ASSERT_UNKNOWN( (op != LdpOp) & (op != LdvOp) );

    // first operator should be a BeginOp and NumRes( BeginOp ) > 0
    num_var_rec_ += NumRes(op);
    CPPAD_ASSERT_UNKNOWN( num_var_rec_ > 0 );

    // index of last variable corresponding to this operation
    // (if NumRes(op) > 0)
    CPPAD_ASSERT_KNOWN(
        (size_t) std::numeric_limits<addr_t>::max() >= num_var_rec_ - 1,
        "cppad_tape_addr_type maximum value has been exceeded"
    )

    return static_cast<addr_t>( num_var_rec_ - 1 );
}

/*!
Put next LdpOp or LdvOp operator in operation sequence (special cases).

This sets the op code for the next operation in this recording.
This call must be followed by putting the corresponding
\verbatim
    NumArg(op)
\endverbatim
argument indices in the recording.

\param op
Is the op code corresponding to the the operation that is being
recorded (which must be LdpOp or LdvOp).

\return
The return value is the index of the primary (last) variable
corresponding to the result of this operation.
The number of variables corresponding to the operation is given by
\verbatim
    NumRes(op)
\endverbatim
which must be one for this operation.
With each call to PutLoadOp or PutOp,
the return index increases by the number of variables corresponding
to this call to the call.
This index starts at zero after the default constructor.

\par num_var_load_rec()
The return value for <code>num_var_load_rec()</code>
increases by one after each call to this function.
*/
template <class Base>
addr_t recorder<Base>::PutLoadOp(OpCode op)
{   size_t i    = op_vec_.extend(1);
    CPPAD_ASSERT_KNOWN(
        (abort_op_index_ == 0) || (abort_op_index_ != i),
        "This is the abort operator index specified by "
        "Independent(x, abort_op_index)."
    );
    op_vec_[i]  = op;
    CPPAD_ASSERT_UNKNOWN( op_vec_.size() == i + 1 );
    CPPAD_ASSERT_UNKNOWN( (op == LdpOp) | (op == LdvOp) );

    // first operator should be a BeginOp and NumRes( BeginOp ) > 0
    num_var_rec_ += NumRes(op);
    CPPAD_ASSERT_UNKNOWN( num_var_rec_ > 0 );

    // count this vecad load operation
    num_var_load_rec_++;

    // index of last variable corresponding to this operation
    // (if NumRes(op) > 0)
    CPPAD_ASSERT_KNOWN(
        (size_t) std::numeric_limits<addr_t>::max() >= num_var_rec_ - 1,
        "cppad_tape_addr_type maximum value has been exceeded"
    )
    return static_cast<addr_t>( num_var_rec_ - 1 );
}

/*!
Put a dynamic parameter at the end of the vector for all parameters.

\param par
is value of dynamic parameter to be placed at the end of the vector.

\param op
is the operator for this dynamic parameter.
There are no arguments to this operator, so numarg(op) == 0.

\return
is the index in all_par_vec_ corresponding to this dynamic parameter value.
*/
template <class Base>
addr_t recorder<Base>::put_dyn_par(const Base &par, op_code_dyn op)
{   // independent parameters come first
    CPPAD_ASSERT_UNKNOWN(
        op == ind_dyn || op == result_dyn || op == atom_dyn
    );
    CPPAD_ASSERT_UNKNOWN( num_arg_dyn(op) == 0 );
    all_par_vec_.push_back( par );
    dyn_par_is_.push_back(true);
    dyn_par_op_.push_back( opcode_t(op) );
    return static_cast<addr_t>( all_par_vec_.size() - 1 );
}
/*!
Put a dynamic parameter at the end of the vector for all parameters.

\param par
is value of dynamic parameter to be placed at the end of the vector.

\param op
is the operator for this dynamic parameter.
There is one argument to this operator, so numarg(op) == 1.

\param arg0
this is the argument to the operator represented
as an index in the all_par_vec_ vector.

\return
is the index in all_par_vec_ corresponding to this dynamic parameter value.
*/
template <class Base>
addr_t recorder<Base>::put_dyn_par(
    const Base &par, op_code_dyn op, addr_t arg0
)
{   // independent parameters come first
    CPPAD_ASSERT_UNKNOWN( num_arg_dyn(op) == 1 );
    all_par_vec_.push_back( par );
    dyn_par_is_.push_back(true);
    dyn_par_op_.push_back( opcode_t(op) );
    dyn_par_arg_.push_back(arg0);
    return static_cast<addr_t>( all_par_vec_.size() - 1 );
}
/*!
Put a dynamic parameter at the end of the vector for all parameters.

\param par
is value of dynamic parameter to be placed at the end of the vector.

\param op
is the operator for this dynamic parameter.
There are two arguments to this operator, so numarg(op) == 2.

\param arg0
this is the first argument to the operator represented
as an index in the all_par_vec_ vector.

\param arg1
this is the second argument to the operator represented
as an index in the all_par_vec_ vector.
One of the two arguments must be a dynamic parameter.

\return
is the index in all_par_vec_ corresponding to this dynamic parameter value.
*/
template <class Base>
addr_t recorder<Base>::put_dyn_par(
    const Base &par, op_code_dyn op, addr_t arg0, addr_t arg1
)
{   // independent parameters come first
    CPPAD_ASSERT_UNKNOWN( num_arg_dyn(op) == 2 );
    all_par_vec_.push_back( par );
    dyn_par_is_.push_back(true);
    dyn_par_op_.push_back( opcode_t(op) );
    dyn_par_arg_.push_back(arg0);
    dyn_par_arg_.push_back(arg1);
    return static_cast<addr_t>( all_par_vec_.size() - 1 );
}
/*!
Put a dynamic parameter, that is result of conditional expression,
at the end of the vector for all parameters.

\param par
is value of dynamic parameter to be placed at the end of the vector.

\param cop
is the operator comparison operator; i.e., Lt, Le, Eq, Ge, Gt, Ne.

\param left
is the left argument in conditional expression (which is a parameter).

\param right
is the right argument in conditional expression (which is a parameter).

\param if_true
is the if_true argument in conditional expression (which is a parameter).

\param if_false
is the if_false argument in conditional expression (which is a parameter).

\return
is the index in all_par_vec_ corresponding to this dynamic parameter value.
*/
template <class Base>
addr_t recorder<Base>::put_dyn_cond_exp(const Base &par, CompareOp cop,
    addr_t left, addr_t right, addr_t if_true, addr_t if_false
)
{   // independent parameters come first
    CPPAD_ASSERT_UNKNOWN( num_arg_dyn(cond_exp_dyn) == 5 );
    addr_t ret = addr_t( all_par_vec_.size() );
    all_par_vec_.push_back( par );
    dyn_par_is_.push_back(true);
    dyn_par_op_.push_back( opcode_t(cond_exp_dyn) );
    dyn_par_arg_.push_back( addr_t(cop) );
    dyn_par_arg_.push_back(left);
    dyn_par_arg_.push_back(right);
    dyn_par_arg_.push_back(if_true);
    dyn_par_arg_.push_back(if_false);
    return ret;
}
// ---------------------------------------------------------------------------
/*!
Puts a vector of arguments at the end of the current dynamic parameter tape

\param arg_vec [in]
is the vector of values to be added at the end of the tape.
*/
template <class Base>
void recorder<Base>::put_dyn_arg_vec(const pod_vector<addr_t>& arg_vec)
{   for(size_t i = 0; i < arg_vec.size(); ++i)
        dyn_par_arg_.push_back( arg_vec[i] );
}

// ---------------------------------------------------------------------------
/*!
Find or add a constant parameter to the current vector of all parameters.

\param par
is the parameter to be found or placed in the vector of parameters.

\return
is the index in the parameter vector corresponding to this parameter value.
This value is not necessarily placed at the end of the vector
(because values that are identically equal may be reused).
*/
template <class Base>
addr_t recorder<Base>::put_con_par(const Base &par)
{
# ifndef NDEBUG
    // index zero is used to signify that a value is not a parameter;
    // i.e., it is a variable.
    if( all_par_vec_.size() == 0 )
        CPPAD_ASSERT_UNKNOWN( isnan(par) );
# endif
    // ---------------------------------------------------------------------
    // check for a match with a previous parameter
    //
    // get hash code for this value
    size_t code  = static_cast<size_t>( hash_code(par) );

    // current index in all_par_vec_ corresponding to this hash code
    size_t index = static_cast<size_t>( par_hash_table_[code] );

    // check if the old parameter matches the new one
    if( (0 < index) & (index < all_par_vec_.size()) )
    {   if( ! dyn_par_is_[index] )
            if( IdenticalEqualCon(all_par_vec_[index], par) )
                return static_cast<addr_t>( index );
    }
    // ---------------------------------------------------------------------
    // put paramerter in all_par_vec_ and replace hash entry for this codee
    //
    index = all_par_vec_.size();
    all_par_vec_.push_back( par );
    dyn_par_is_.push_back(false);
    //
    // change the hash table for this code to point to new value
    par_hash_table_[code] = static_cast<addr_t>( index );
    //
    // return the parameter index
    CPPAD_ASSERT_KNOWN(
        static_cast<size_t>( std::numeric_limits<addr_t>::max() ) >= index,
        "cppad_tape_addr_type maximum value has been exceeded"
    )
    return static_cast<addr_t>( index );
}
// -------------------------- PutArg --------------------------------------
/*!
Prototype for putting operation argument indices in the recording.

The following syntax
\verbatim
    rec.PutArg(arg0)
    rec.PutArg(arg0, arg1)
    .
    .
    .
    rec.PutArg(arg0, arg1, ..., arg5)
\endverbatim
places the values passed to PutArg at the current end of the
operation argument indices for the recording.
 arg0 comes before arg1, etc.
The proper number of operation argument indices
corresponding to the operation code op is given by
\verbatim
    NumArg(op)
\endverbatim
The number of the operation argument indices starts at zero
after the default constructor.
*/
inline void prototype_put_arg(void)
{   // This routine should not be called
    CPPAD_ASSERT_UNKNOWN(false);
}
/*!
Put one operation argument index in the recording

\param arg0
The operation argument index

\copydetails prototype_put_arg
*/
template <class Base>
void recorder<Base>::PutArg(addr_t arg0)
{
    size_t i      =  arg_vec_.extend(1);
    arg_vec_[i]   =  arg0;
    CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == i + 1 );
}
/*!
Put two operation argument index in the recording

\param arg0
First operation argument index.

\param arg1
Second operation argument index.

\copydetails prototype_put_arg
*/
template <class Base>
void recorder<Base>::PutArg(addr_t arg0, addr_t arg1)
{
    size_t i      =  arg_vec_.extend(2);
    arg_vec_[i++] =  arg0;
    arg_vec_[i]   =  arg1;
    CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == i + 1 );
}
/*!
Put three operation argument index in the recording

\param arg0
First operation argument index.

\param arg1
Second operation argument index.

\param arg2
Third operation argument index.

\copydetails prototype_put_arg
*/
template <class Base>
void recorder<Base>::PutArg(addr_t arg0, addr_t arg1, addr_t arg2)
{
    size_t i      =  arg_vec_.extend(3);
    arg_vec_[i++] =  arg0;
    arg_vec_[i++] =  arg1;
    arg_vec_[i]   =  arg2;
    CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == i + 1 );
}
/*!
Put four operation argument index in the recording

\param arg0
First operation argument index.

\param arg1
Second operation argument index.

\param arg2
Third operation argument index.

\param arg3
Fourth operation argument index.

\copydetails prototype_put_arg
*/
template <class Base>
void recorder<Base>::PutArg(addr_t arg0, addr_t arg1, addr_t arg2,
    addr_t arg3)
{
    size_t i      =  arg_vec_.extend(4);
    arg_vec_[i++] =  arg0;
    arg_vec_[i++] =  arg1;
    arg_vec_[i++] =  arg2;
    arg_vec_[i]   =  arg3;
    CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == i + 1 );

}
/*!
Put five operation argument index in the recording

\param arg0
First operation argument index.

\param arg1
Second operation argument index.

\param arg2
Third operation argument index.

\param arg3
Fourth operation argument index.

\param arg4
Fifth operation argument index.

\copydetails prototype_put_arg
*/
template <class Base>
void recorder<Base>::PutArg(addr_t arg0, addr_t arg1, addr_t arg2,
    addr_t arg3, addr_t arg4)
{
    size_t i      =  arg_vec_.extend(5);
    arg_vec_[i++] =  arg0;
    arg_vec_[i++] =  arg1;
    arg_vec_[i++] =  arg2;
    arg_vec_[i++] =  arg3;
    arg_vec_[i]   =  arg4;
    CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == i + 1 );

}
/*!
Put six operation argument index in the recording

\param arg0
First operation argument index.

\param arg1
Second operation argument index.

\param arg2
Third operation argument index.

\param arg3
Fourth operation argument index.

\param arg4
Fifth operation argument index.

\param arg5
Sixth operation argument index.

\copydetails prototype_put_arg
*/
template <class Base>
void recorder<Base>::PutArg(addr_t arg0, addr_t arg1, addr_t arg2,
    addr_t arg3, addr_t arg4, addr_t arg5)
{
    size_t i      =  arg_vec_.extend(6);
    arg_vec_[i++] =  arg0;
    arg_vec_[i++] =  arg1;
    arg_vec_[i++] =  arg2;
    arg_vec_[i++] =  arg3;
    arg_vec_[i++] =  arg4;
    arg_vec_[i]   =  arg5;
    CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == i + 1 );
}
// --------------------------------------------------------------------------
/*!
Reserve space for arguments, but delay placing values there.

\param n_arg
number of arguements to reserve space for

\return
is the index in the argument vector corresponding to the
first of the arguments being reserved.
*/
template <class Base>
size_t recorder<Base>::ReserveArg(size_t n_arg)
{
    size_t i      =  arg_vec_.extend(n_arg);
    CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == i + n_arg );
    return i;
}

/*!
\brief
Replace an argument value in the recording
(intended to fill in reserved values).

\param i_arg
is the index, in argument vector, for the value that is replaced.

\param value
is the new value for the argument with the specified index.
*/
template <class Base>
void recorder<Base>::ReplaceArg(size_t i_arg, addr_t value)
{   arg_vec_[i_arg] =  value; }
// --------------------------------------------------------------------------
/*!
Put a character string in the text for this recording.

\param text
is a '\\0' terminated character string that is to be put in the
vector of characters corresponding to this recording.
The terminator '\\0' will be included.

\return
is the offset with in the text vector for this recording at which
the character string starts.
*/
template <class Base>
addr_t recorder<Base>::PutTxt(const char *text)
{
    // determine length of the text including terminating '\0'
    size_t n = 0;
    while( text[n] != '\0' )
        n++;
    CPPAD_ASSERT_UNKNOWN( n <= 1000 );
    n++;
    CPPAD_ASSERT_UNKNOWN( text[n-1] == '\0' );

    // copy text including terminating '\0'
    size_t i = text_vec_.extend(n);
    size_t j;
    for(j = 0; j < n; j++)
        text_vec_[i + j] = text[j];
    CPPAD_ASSERT_UNKNOWN( text_vec_.size() == i + n );

    CPPAD_ASSERT_KNOWN(
        size_t( std::numeric_limits<addr_t>::max() ) >= i,
        "cppad_tape_addr_type maximum value has been exceeded"
    );
    //
    return static_cast<addr_t>( i );
}

} } // END_CPPAD_LOCAL_NAMESPACE

// ----------------------------------------------------------------------------
// member function implementations
# include <cppad/local/record/put_var_vecad.hpp>
# include <cppad/local/record/put_dyn_atomic.hpp>
# include <cppad/local/record/put_var_atomic.hpp>
# include <cppad/local/record/cond_exp.hpp>
# include <cppad/local/record/comp_op.hpp>

# endif
