#ifndef CPPAD_CG_TIME_DIFF_INCLUDED
#define CPPAD_CG_TIME_DIFF_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

namespace CppAD {
namespace cg {

// ----------------------------------------------------------------------
// forward mode routine called by CppAD for  y = f(x, dxdt, t)

template<class Base>
bool time_diff_forward(size_t id,
                       size_t order,
                       size_t n,
                       size_t m,
                       const CppAD::vector<bool>& vx,
                       CppAD::vector<bool>& vzy,
                       const CppAD::vector<CG<Base> >& tx,
                       CppAD::vector<CG<Base> >& tzy) {
    CPPADCG_ASSERT_UNKNOWN(n == 3); // [x, dxdt, t]
    CPPADCG_ASSERT_UNKNOWN(m == 1);
    CPPADCG_ASSERT_UNKNOWN(tx.size() >= (order + 1) * n);
    CPPADCG_ASSERT_UNKNOWN(tzy.size() >= (order + 1) * m);

    size_t n_order = order + 1;
    const size_t xIndex = 0; // index of the variable in the argument list
    const size_t dxdtIndex = 1; // index of the time derivative variable in the argument list
    const size_t timeIndex = 2; // index of the time variable in the argument list

    // check if this is during the call to time_var(id, ax, ay)
    if (vx.size() > 0) {
        CPPADCG_ASSERT_UNKNOWN(vx.size() >= n);
        CPPADCG_ASSERT_UNKNOWN(vzy.size() >= m);

        vzy[0] = vx[0] || vx[1] || vx[2];
    }

    if (order == 0) {
        tzy[0] = tx[0];
    } else if (order == 1) {
        const CG<Base>& ttime = tx[timeIndex * n_order + order]; //
        const CG<Base>& txx = tx[xIndex * n_order + order]; //
        CPPADCG_ASSERT_UNKNOWN(ttime.isParameter());
        CPPADCG_ASSERT_UNKNOWN(txx.isParameter());
        if (ttime.getValue() > 0) {
            CPPADCG_ASSERT_UNKNOWN(txx.getValue() == 0);
            tzy[1] = ttime * tx[dxdtIndex * n_order + 0]; // transform x(t) into dx(t)/dt
        } else {
            tzy[1] = txx; // do nothing
        }

    } else {
        return false; // not implemented
    }

    // All orders are implemented and there are no possible errors
    return true;
}
// ----------------------------------------------------------------------
// reverse mode routine called by CppAD for  y = f(x, dxdt, t)

template<class Base>
bool time_diff_reverse(size_t id,
                       size_t order,
                       size_t n,
                       size_t m,
                       const CppAD::vector<CG<Base> >& tx,
                       const CppAD::vector<CG<Base> >& tzy,
                       CppAD::vector<CG<Base> >& px,
                       const CppAD::vector<CG<Base> >& pzy) {

    CPPADCG_ASSERT_UNKNOWN(n == 3); // [x, dxdt, t]
    CPPADCG_ASSERT_UNKNOWN(m == 1);
    CPPADCG_ASSERT_UNKNOWN(tx.size() >= (order + 1) * n);
    CPPADCG_ASSERT_UNKNOWN(tzy.size() >= (order + 1) * m);
    CPPADCG_ASSERT_UNKNOWN(px.size() >= (order + 1) * n);

    CG<Base>* pxx = &px[0];
    CG<Base>* pdxdt = &px[order + 1];
    CG<Base>* pt = &px[2 * (order + 1)];

    //const CG<Base>* txx = &tx[0];
    const CG<Base>* tdxdt = &tx[order + 1];
    //const CG<Base>* tt = &tx[2 * (order + 1)];

    if (order == 0) {
        pxx[0] = pzy[0] * 1.0;
        pdxdt[0] = 0.0;
        pt[0] = pzy[0] * tdxdt[0];
        return true;
    }

    return false; // not implemented yet
}
// ----------------------------------------------------------------------
// forward Jacobian sparsity routine called by CppAD

template<class Base>
bool time_diff_for_jac_sparse(size_t id,
                              size_t n,
                              size_t m,
                              size_t q,
                              const CppAD::vector< std::set<size_t> >& r,
                              CppAD::vector< std::set<size_t> >& s) {
    CPPADCG_ASSERT_UNKNOWN(n == 3);
    CPPADCG_ASSERT_UNKNOWN(m == 1);
    CPPADCG_ASSERT_UNKNOWN(r.size() >= n);
    CPPADCG_ASSERT_UNKNOWN(s.size() >= m);

    // sparsity for z and y are the same as for x
    s[0] = r[0]; // x
    s[0].insert(r[1].begin(), r[1].end()); // dxdt
    s[0].insert(r[2].begin(), r[2].end()); // t

    return true;
}
// ----------------------------------------------------------------------
// reverse Jacobian sparsity routine called by CppAD

template<class Base>
bool time_diff_rev_jac_sparse(size_t id,
                              size_t n,
                              size_t m,
                              size_t q,
                              CppAD::vector< std::set<size_t> >& r,
                              const CppAD::vector< std::set<size_t> >& s) {
    CPPADCG_ASSERT_UNKNOWN(n == 3);
    CPPADCG_ASSERT_UNKNOWN(m == 1);
    CPPADCG_ASSERT_UNKNOWN(r.size() >= n);
    CPPADCG_ASSERT_UNKNOWN(s.size() >= m);

    r[0] = s[0];
    r[2] = s[0];

    return false;
}
// ----------------------------------------------------------------------
// reverse Hessian sparsity routine called by CppAD

template<class Base>
bool time_diff_rev_hes_sparse(size_t id,
                              size_t n,
                              size_t m,
                              size_t q,
                              const CppAD::vector< std::set<size_t> >& r,
                              const CppAD::vector<bool>& s,
                              CppAD::vector<bool>& t,
                              const CppAD::vector< std::set<size_t> >& u,
                              CppAD::vector< std::set<size_t> >& v) {
    return false;
}
// ---------------------------------------------------------------------
// Declare the AD<CG<Base> > routine time_var(id, ax, ay)
template<class Base>
CPPAD_USER_ATOMIC(time_var,
                  std::vector,
                  cg::CG<Base>,
                  CppAD::cg::time_diff_forward<Base>,
                  CppAD::cg::time_diff_reverse<Base>,
                  CppAD::cg::time_diff_for_jac_sparse<Base>,
                  CppAD::cg::time_diff_rev_jac_sparse<Base>,
                  CppAD::cg::time_diff_rev_hes_sparse<Base>)

} // END cg namespace
} // END CppAD namespace

#endif