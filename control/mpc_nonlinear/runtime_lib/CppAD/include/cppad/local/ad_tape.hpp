# ifndef CPPAD_LOCAL_AD_TAPE_HPP
# define CPPAD_LOCAL_AD_TAPE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/define.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL__NAMESPACE

/*!
Class used to hold tape that records AD<Base> operations.

\tparam Base
An <tt>AD<Base></tt> object is used to recording <tt>AD<Base></tt> operations.
*/

template <class Base>
class ADTape {
    // Friends =============================================================

    // classes -------------------------------------------------------------
    friend class AD<Base>;
    friend class ADFun<Base>;
    friend class atomic_base<Base>;
    friend class atomic_three<Base>;
    friend class discrete<Base>;
    friend class VecAD<Base>;
    friend class VecAD_reference<Base>;

    // functions -----------------------------------------------------------
    // PrintFor
    friend void CppAD::PrintFor <Base> (
        const AD<Base>&    flag   ,
        const char*        before ,
        const AD<Base>&    var    ,
        const char*        after
    );
    // CondExpOp
    friend AD<Base> CppAD::CondExpOp <Base> (
        enum CompareOp  cop          ,
        const AD<Base> &left         ,
        const AD<Base> &right        ,
        const AD<Base> &trueCase     ,
        const AD<Base> &falseCase
    );
    // pow
    friend AD<Base> CppAD::pow <Base>
        (const AD<Base> &x, const AD<Base> &y);
    // azmul
    friend AD<Base> CppAD::azmul <Base>
        (const AD<Base> &x, const AD<Base> &y);
    // Parameter
    friend bool CppAD::Parameter     <Base>
        (const AD<Base> &u);
    // Variable
    friend bool CppAD::Variable      <Base>
        (const AD<Base> &u);
    // operators -----------------------------------------------------------
    // arithematic binary operators
# if _MSC_VER && !defined(__clang__)
    // see https://stackoverflow.com/questions/63288453
    template <class Type> friend AD<Type> CppAD::operator * <Type>
        (const AD<Type> &left, const AD<Type> &right);
# else
    friend AD<Base> CppAD::operator * <Base>
        (const AD<Base> &left, const AD<Base> &right);
# endif
    friend AD<Base> CppAD::operator + <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend AD<Base> CppAD::operator - <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend AD<Base> CppAD::operator / <Base>
        (const AD<Base> &left, const AD<Base> &right);

    // comparison operators
# if _MSC_VER && !defined(__clang__)
    template <class Type> friend bool CppAD::operator == <Type>
        (const AD<Type> &left, const AD<Type> &right);
    template <class Type> friend bool CppAD::operator != <Type>
        (const AD<Type> &left, const AD<Type> &right);
# else
    friend bool CppAD::operator == <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool CppAD::operator != <Base>
        (const AD<Base> &left, const AD<Base> &right);
# endif
    friend bool CppAD::operator < <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool CppAD::operator <= <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool CppAD::operator > <Base>
        (const AD<Base> &left, const AD<Base> &right);
    friend bool CppAD::operator >= <Base>
        (const AD<Base> &left, const AD<Base> &right);
    // ======================================================================

// --------------------------------------------------------------------------
private:
    // ----------------------------------------------------------------------
    // private data
    /*!
    Unique identifier for this tape.  It is always greater than
    CPPAD_MAX_NUM_THREADS, and different for every tape (even ones that have
    been deleted). In addition, id_ % CPPAD_MAX_NUM_THREADS is the thread
    number for this tape. Set by Independent and effectively const
    */
    tape_id_t                    id_;
    /// Number of independent variables in this tapes reconding.
    /// Set by Independent and effectively const
    size_t         size_independent_;
    /// This is where the information is recorded.
    local::recorder<Base>              Rec_;
    // ----------------------------------------------------------------------
    // private functions
    //
    // add a parameter to the tape
    addr_t RecordParOp(const AD<Base>& y);

    // see CondExp.h
    void RecordCondExp(
        enum CompareOp  cop           ,
        AD<Base>       &returnValue   ,
        const AD<Base> &left          ,
        const AD<Base> &right         ,
        const AD<Base> &trueCase      ,
        const AD<Base> &falseCase
    );

public:
    // public function only used by CppAD::Independent
    template <class ADBaseVector>
    void Independent(
        ADBaseVector&   x              ,
        size_t          abort_op_index ,
        bool            record_compare ,
        ADBaseVector&   dynamic
    );

};
// ---------------------------------------------------------------------------
// Private functions
//

/*!
Place a parameter in the tape as a variable.

On rare occations it is necessary to place a parameter in the tape; e.g.,
when it is one of the dependent variabes.

\param y
value of the parameter that we are placing in the tape as a variable.

\return
variable index (for this recording) correpsonding to the parameter.

\par 2DO
All these operates are preformed in Rec_, so we should
move this routine from <tt>ADTape<Base></tt> to <tt>recorder<Base></tt>.
*/
template <class Base>
addr_t ADTape<Base>::RecordParOp(const AD<Base>& y)
{   CPPAD_ASSERT_UNKNOWN( NumRes(ParOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumArg(ParOp) == 1 );
    addr_t z_taddr = Rec_.PutOp(ParOp);
    if( Dynamic(y) )
    {   addr_t ind  = y.taddr_;
        Rec_.PutArg(ind);
    }
    else
    {   addr_t ind  = Rec_.put_con_par(y.value_);
        Rec_.PutArg(ind);
    }
    return z_taddr;
}

} } // END_CPPAD_LOCAL_NAMESPACE

# endif
