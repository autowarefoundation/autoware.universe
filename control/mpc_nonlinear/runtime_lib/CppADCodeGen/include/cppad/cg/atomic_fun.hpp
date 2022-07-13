#ifndef CPPAD_CG_ATOMIC_FUN_INCLUDED
#define CPPAD_CG_ATOMIC_FUN_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *    Copyright (C) 2018 Joao Leal
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

/**
 * An atomic function for source code generation
 *
 * @author Joao Leal
 */
template <class Base>
class CGAtomicFun : public CGAbstractAtomicFun<Base> {
protected:
    using CGB = CG<Base>;
protected:
    atomic_base<Base>& atomicFun_;
    const CppAD::vector<Base> xSparsity_; // independent vector used to determine sparsity patterns
public:

    /**
     * Creates a new atomic function wrapper that is responsible for
     * defining the dependencies to calls of a user atomic function.
     *
     * @param atomicFun The atomic function to the called by the compiled
     *                  source.
     * @param xSparsity Default independent vector used to determine sparsity patterns
     *                  when the provided independent vector using the CG data type does
     *                  not have all values defined.
     * @param standAlone Whether or not forward and reverse function calls
     *                   do not require the Taylor coefficients for the
     *                   dependent variables (ty) and the previous
     *                   evaluation of other forward/reverse modes.
     */
    CGAtomicFun(atomic_base<Base>& atomicFun,
                const CppAD::vector<Base>& xSparsity,
                bool standAlone = false) :
        CGAbstractAtomicFun<Base>(atomicFun.atomic_name(), standAlone),
        atomicFun_(atomicFun),
        xSparsity_(xSparsity) {
    }

    CGAtomicFun(atomic_base<Base>& atomicFun,
                ArrayView<const Base> xSparsity,
                bool standAlone = false) :
            CGAtomicFun(atomicFun, values(xSparsity), standAlone) {
    }

    CGAtomicFun(atomic_base<Base>& atomicFun,
                ArrayView<const CppAD::AD<Base>> xSparsity,
                bool standAlone = false) :
            CGAtomicFun(atomicFun, values(xSparsity), standAlone) {
    }

    virtual ~CGAtomicFun() = default;

    template <class ADVector>
    void operator()(const ADVector& ax,
                    ADVector& ay,
                    size_t id = 0) {
        this->CGAbstractAtomicFun<Base>::operator()(ax, ay, id);
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        CppAD::vector<std::set<size_t> >& s,
                        const CppAD::vector<CGB>& x) override {
        return atomicFun_.for_sparse_jac(q, r, s, sparsityIndeps(x));
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        CppAD::vector<std::set<size_t> >& s) override {
        return atomicFun_.for_sparse_jac(q, r, s);
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<bool>& r,
                        CppAD::vector<bool>& s,
                        const CppAD::vector<CGB>& x) override {
        return atomicFun_.for_sparse_jac(q, r, s, sparsityIndeps(x));
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<bool>& r,
                        CppAD::vector<bool>& s) override {
        return atomicFun_.for_sparse_jac(q, r, s);
    }

    bool rev_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& rt,
                        CppAD::vector<std::set<size_t> >& st,
                        const CppAD::vector<CGB>& x) override {
        return atomicFun_.rev_sparse_jac(q, rt, st, sparsityIndeps(x));
    }

    bool rev_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& rt,
                        CppAD::vector<std::set<size_t> >& st) override {
        return atomicFun_.rev_sparse_jac(q, rt, st);
    }

    bool rev_sparse_jac(size_t q,
                        const CppAD::vector<bool>& rt,
                        CppAD::vector<bool>& st,
                        const CppAD::vector<CGB>& x) override {
        return atomicFun_.rev_sparse_jac(q, rt, st, sparsityIndeps(x));
    }

    bool rev_sparse_jac(size_t q,
                        const CppAD::vector<bool>& rt,
                        CppAD::vector<bool>& st) override {
        return atomicFun_.rev_sparse_jac(q, rt, st);
    }

    bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                        const CppAD::vector<bool>& s,
                        CppAD::vector<bool>& t,
                        size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        const CppAD::vector<std::set<size_t> >& u,
                        CppAD::vector<std::set<size_t> >& v,
                        const CppAD::vector<CGB>& x) override {
        return atomicFun_.rev_sparse_hes(vx, s, t, q, r, u, v, sparsityIndeps(x));
    }

    bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                        const CppAD::vector<bool>& s,
                        CppAD::vector<bool>& t,
                        size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        const CppAD::vector<std::set<size_t> >& u,
                        CppAD::vector<std::set<size_t> >& v) override {
        return atomicFun_.rev_sparse_hes(vx, s, t, q, r, u, v);
    }

    bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                        const CppAD::vector<bool>& s,
                        CppAD::vector<bool>& t,
                        size_t q,
                        const CppAD::vector<bool>& r,
                        const CppAD::vector<bool>& u,
                        CppAD::vector<bool>& v,
                        const CppAD::vector<CGB>& x) override {
        return atomicFun_.rev_sparse_hes(vx, s, t, q, r, u, v, sparsityIndeps(x));
    }

    bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                        const CppAD::vector<bool>& s,
                        CppAD::vector<bool>& t,
                        size_t q,
                        const CppAD::vector<bool>& r,
                        const CppAD::vector<bool>& u,
                        CppAD::vector<bool>& v) override {
        return atomicFun_.rev_sparse_hes(vx, s, t, q, r, u, v);
    }

protected:

    void zeroOrderDependency(const CppAD::vector<bool>& vx,
                             CppAD::vector<bool>& vy,
                             const CppAD::vector<CGB>& x) override {
        using CppAD::vector;

        size_t m = vy.size();
        size_t n = vx.size();

        vector<std::set<size_t> > rt(m);
        for (size_t j = 0; j < m; j++) {
            rt[j].insert(j);
        }
        vector<std::set<size_t> > st(n);

        bool ok = rev_sparse_jac(m, rt, st, x);
        if (!ok)
            throw CGException("False returned from rev_sparse_jac() in the atomic function \"", this->atomic_name(), "\".");

        for (size_t j = 0; j < n; j++) {
            for (size_t i : st[j]) {
                if (vx[j]) {
                    vy[i] = true;
                }
            }
        }
    }

    bool atomicForward(size_t q,
                       size_t p,
                       const CppAD::vector<Base>& tx,
                       CppAD::vector<Base>& ty) override {
        CppAD::vector<bool> vx, vy;
        return atomicFun_.forward(q, p, vx, vy, tx, ty);
    }

    bool atomicReverse(size_t p,
                       const CppAD::vector<Base>& tx,
                       const CppAD::vector<Base>& ty,
                       CppAD::vector<Base>& px,
                       const CppAD::vector<Base>& py) override {
        return atomicFun_.reverse(p, tx, ty, px, py);
    }

private:
    inline CppAD::vector<Base> sparsityIndeps(const CppAD::vector<CGB>& x) const {
        CPPADCG_ASSERT_UNKNOWN(x.size() == xSparsity_.size());

        size_t n = x.size();
        CppAD::vector<Base> out(n);
        for (size_t i = 0; i < n; ++i) {
            if (x[i].isValueDefined()) {
                out[i] = x[i].getValue();
            } else {
                out = xSparsity_;
                break;
            }
        }

        return out;
    }

    inline static CppAD::vector<Base> values(ArrayView<const CppAD::AD<Base>> x) {
        CppAD::vector<Base> out(x.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = CppAD::Value(CppAD::Var2Par(x[i]));
        }
        return out;
    }

    inline static CppAD::vector<Base> values(ArrayView<const Base> x) {
        CppAD::vector<Base> out(x.size());
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = x[i];
        }
        return out;
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
