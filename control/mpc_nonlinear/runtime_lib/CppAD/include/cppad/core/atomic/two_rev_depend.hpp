# ifndef CPPAD_CORE_ATOMIC_TWO_REV_DEPEND_HPP
# define CPPAD_CORE_ATOMIC_TWO_REV_DEPEND_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/two_rev_depend.hpp
Third generation atomic type computation.
*/
/*!
Link from atomic_two to reverse dependency calculation

\param parameter_x [in]
is the value of the parameters in the corresponding function call
afun(ax, ay).

\param type_x [in]
is the type for each component of ax in the corresponding function call
afun(ax, ay).

\param depend_x [out]
specifies which components of x affect values of interest.

\param depend_y [in]
specifies which components of y affect values of interest.
*/
// BEGIN_PROTOTYPE
template <class Base>
bool atomic_base<Base>::rev_depend(
    const vector<Base>&         parameter_x ,
    const vector<ad_type_enum>& type_x      ,
    vector<bool>&               depend_x    ,
    const vector<bool>&         depend_y    )
// END_PROTOTYPE
{   bool ok = true;
    CPPAD_ASSERT_UNKNOWN( depend_x.size() == parameter_x.size() );
    size_t n = depend_x.size();
    size_t m = depend_y.size();
    //
    size_t thread = thread_alloc::thread_num();
    allocate_work(thread);
    //
    if( sparsity_ == pack_sparsity_enum )
    {   vectorBool& rt ( work_[thread]->pack_r );
        vectorBool& st ( work_[thread]->pack_s );
        //
        st.resize(n * 1 );
        rt.resize(m * 1 );
        for(size_t i = 0; i < m; ++i)
            rt[i] = depend_y[i];
        ok = rev_sparse_jac(1, rt, st, parameter_x);
        if( ! ok )
            ok = rev_sparse_jac(1, rt, st);
        if( ! ok )
            return false;
        for(size_t j = 0; j < n; ++j)
            depend_x[j] = st[j];
    }
    else if( sparsity_ == bool_sparsity_enum )
    {
        ok = rev_sparse_jac(1, depend_y, depend_x, parameter_x);
        if( ! ok )
            ok = rev_sparse_jac(m, depend_y, depend_x);
        if( ! ok )
            return false;
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( sparsity_ == set_sparsity_enum );
        vector< std::set<size_t> >& rt ( work_[thread]->set_r );
        vector< std::set<size_t> >& st ( work_[thread]->set_s );
        rt.resize(m);
        st.resize(n);
        for(size_t i = 0; i < m; ++i)
        {   if( depend_y[i] )
                rt[i].insert(0);
        }
        ok = rev_sparse_jac(m, rt, st, parameter_x);
        if( ! ok )
            ok = rev_sparse_jac(m, rt, st);
        if( ! ok )
            return false;
        for(size_t j = 0; j < n; ++j)
            depend_x[j] = ! st[j].empty();
    }
    return ok;
}

} // END_CPPAD_NAMESPACE

# endif
