#ifndef CPPAD_CG_ATOMIC_GENERIC_MODEL_INCLUDED
#define CPPAD_CG_ATOMIC_GENERIC_MODEL_INCLUDED
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
 * An atomic function that uses a compiled model
 *
 * @author Joao Leal
 */
template <class Base>
class CGAtomicGenericModel : public atomic_base<Base> {
protected:
    GenericModel<Base>& model_;
public:

    /**
     * Creates a new atomic function wrapper that is responsible for
     * calling the appropriate methods of the compiled model.
     *
     * @param model The compiled model.
     */
    CGAtomicGenericModel(GenericModel<Base>& model) :
        atomic_base<Base>(model.getName()),
        model_(model) {
        this->option(CppAD::atomic_base<Base>::set_sparsity_enum);
    }

    virtual ~CGAtomicGenericModel() = default;

    template <class ADVector>
    void operator()(const ADVector& ax, ADVector& ay, size_t id = 0) {
        this->atomic_base<Base>::operator()(ax, ay, id);
    }

    bool forward(size_t q,
                 size_t p,
                 const CppAD::vector<bool>& vx,
                 CppAD::vector<bool>& vy,
                 const CppAD::vector<Base>& tx,
                 CppAD::vector<Base>& ty) override {
        if (p == 0) {
            model_.ForwardZero(vx, vy, tx, ty);
            return true;
        } else if (p == 1) {
            model_.ForwardOne(tx, ty);
            return true;
        }

        return false;
    }

    bool reverse(size_t p,
                 const CppAD::vector<Base>& tx,
                 const CppAD::vector<Base>& ty,
                 CppAD::vector<Base>& px,
                 const CppAD::vector<Base>& py) override {
        if (p == 0) {
            model_.ReverseOne(tx, ty, px, py);
            return true;
        } else if (p == 1) {
            model_.ReverseTwo(tx, ty, px, py);
            return true;
        }

        return false;
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        CppAD::vector<std::set<size_t> >& s,
                        const CppAD::vector<Base>& x) override {
        return for_sparse_jac(q, r, s);
    }

    bool for_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        CppAD::vector<std::set<size_t> >& s) override {
        size_t n = model_.Domain();
        size_t m = model_.Range();
        for (size_t i = 0; i < m; i++) {
            s[i].clear();
        }

        const std::vector<std::set<size_t> > jacSparsity = model_.JacobianSparsitySet();

        // S(x) =  f'(x) * R
        CppAD::cg::multMatrixMatrixSparsity(jacSparsity, r, s, m, n, q);

        return true;
    }

    bool rev_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& rT,
                        CppAD::vector<std::set<size_t> >& sT,
                        const CppAD::vector<Base>& x) override {
        return rev_sparse_jac(q, rT, sT);
    }

    bool rev_sparse_jac(size_t q,
                        const CppAD::vector<std::set<size_t> >& rT,
                        CppAD::vector<std::set<size_t> >& sT) override {
        size_t n = model_.Domain();
        size_t m = model_.Range();
        for (size_t i = 0; i < n; i++) {
            sT[i].clear();
        }

        const std::vector<std::set<size_t> > jacSparsity = model_.JacobianSparsitySet();

        // S(x)^T = ( R * f'(x) )^T = f'(x)^T * R^T
        CppAD::cg::multMatrixTransMatrixSparsity(jacSparsity, rT, sT, m, n, q);

        return true;
    }

    bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                        const CppAD::vector<bool>& s,
                        CppAD::vector<bool>& t,
                        size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        const CppAD::vector<std::set<size_t> >& u,
                        CppAD::vector<std::set<size_t> >& v,
                        const CppAD::vector<Base>& x) override {
        return rev_sparse_hes(vx, s, t, q, r, u, v);
    }

    bool rev_sparse_hes(const CppAD::vector<bool>& vx,
                        const CppAD::vector<bool>& s,
                        CppAD::vector<bool>& t,
                        size_t q,
                        const CppAD::vector<std::set<size_t> >& r,
                        const CppAD::vector<std::set<size_t> >& u,
                        CppAD::vector<std::set<size_t> >& v) override {
        size_t n = model_.Domain();
        size_t m = model_.Range();

        for (size_t i = 0; i < n; i++) {
            v[i].clear();
        }

        const std::vector<std::set<size_t> > jacSparsity = model_.JacobianSparsitySet();

        /**
         *  V(x)  =  f'^T(x) U(x)  +  Sum(  s(x)i  f''(x)  R   )
         */
        // f'^T(x) U(x)
        CppAD::cg::multMatrixTransMatrixSparsity(jacSparsity, u, v, m, n, q);

        // Sum(  s(x)i  f''(x)  R   )
        bool allSelected = true;
        for (size_t i = 0; i < m; i++) {
            if (!s[i]) {
                allSelected = false;
                break;
            }
        }

        if (allSelected) {
            // TODO: use reverseTwo sparsity instead of the HessianSparsity (they can be different!!!)
            std::vector<std::set<size_t> > sparsitySF2R = model_.HessianSparsitySet(); // f''(x)
            sparsitySF2R.resize(n);
            CppAD::cg::multMatrixMatrixSparsity(sparsitySF2R, r, v, n, n, q); // f''(x) * R
        } else {
            std::vector<std::set<size_t> > sparsitySF2R(n);
            for (size_t i = 0; i < m; i++) {
                if (s[i]) {
                    CppAD::cg::addMatrixSparsity(model_.HessianSparsitySet(i), sparsitySF2R); // f''_i(x)
                }
            }
            CppAD::cg::multMatrixMatrixSparsity(sparsitySF2R, r, v, n, n, q); // f''(x) * R
        }

        /**
         * S(x) * f'(x)
         */
        for (size_t i = 0; i < m; i++) {
            if (s[i]) {
                for (size_t j : jacSparsity[i]) {
                    t[j] = true;
                }
            }
        }

        return true;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
