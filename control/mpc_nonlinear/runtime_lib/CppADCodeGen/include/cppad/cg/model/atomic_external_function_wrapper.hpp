#ifndef CPPAD_CG_ATOMIC_EXTERNAL_FUNCTION_INCLUDED
#define CPPAD_CG_ATOMIC_EXTERNAL_FUNCTION_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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

template<class Base>
class AtomicExternalFunctionWrapper : public ExternalFunctionWrapper<Base> {
private:
    atomic_base<Base>* atomic_;
public:

    inline AtomicExternalFunctionWrapper(atomic_base<Base>& atomic) :
        atomic_(&atomic) {
    }

    inline virtual ~AtomicExternalFunctionWrapper() = default;

    bool forward(FunctorGenericModel<Base>& libModel,
                 int q,
                 int p,
                 const Array tx[],
                 Array& ty) override {
        size_t m = ty.size;
        size_t n = tx[0].size;

        CppAD::vector<bool> vx, vy;

        convert(tx, libModel._tx, n, p, p + 1);

        size_t ty_size = m * (p + 1);
        libModel._ty.resize(ty_size);

        std::fill(&libModel._ty[0], &libModel._ty[0] + ty_size, Base(0));

        bool ret = atomic_->forward(q, p, vx, vy, libModel._tx, libModel._ty);

        convertAdd(libModel._ty, ty, m, p, p);

        return ret;
    }

    bool reverse(FunctorGenericModel<Base>& libModel,
                 int p,
                 const Array tx[],
                 Array& px,
                 const Array py[]) override {
        size_t m = py[0].size;
        size_t n = tx[0].size;

        convert(tx, libModel._tx, n, p, p + 1);

        libModel._ty.resize(m * (p + 1));
        std::fill(&libModel._ty[0], &libModel._ty[0] + libModel._ty.size(), Base(0));

        convert(py, libModel._py, m, p, p + 1);

        size_t px_size = n * (p + 1);
        libModel._px.resize(px_size);

        std::fill(&libModel._px[0], &libModel._px[0] + px_size, Base(0));

#ifndef NDEBUG
        if (libModel._evalAtomicForwardOne4CppAD) {
            // only required in order to avoid an issue with a validation inside CppAD
            CppAD::vector<bool> vx, vy;
            if (!atomic_->forward(p, p, vx, vy, libModel._tx, libModel._ty))
                return false;
        }
#endif

        bool ret = atomic_->reverse(p, libModel._tx, libModel._ty, libModel._px, libModel._py);

        convertAdd(libModel._px, px, n, p, 0); // k=0 for both p=0 and p=1

        return ret;
    }

private:

    inline void convert(const Array from[],
                        CppAD::vector<Base>& to,
                        size_t n,
                        size_t p,
                        size_t kmax) {
        size_t p1 = p + 1;
        to.resize(n * p1);

        for (size_t k = 0; k < kmax; k++) {
            Base* values = static_cast<Base*> (from[k].data);
            if (from[k].sparse) {
                if (p == 0) {
                    std::fill(&to[0], &to[0] + n, Base(0));
                } else {
                    for (size_t j = 0; j < n; j++) {
                        to[j * p1 + k] = Base(0);
                    }
                }

                for (size_t e = 0; e < from[k].nnz; e++) {
                    size_t j = from[k].idx[e];
                    to[j * p1 + k] = values[e];
                }
            } else {
                for (size_t j = 0; j < n; j++) {
                    to[j * p1 + k] = values[j];
                }
            }
        }
    }

    inline void convertAdd(const CppAD::vector<Base>& from,
                           Array& to,
                           size_t n,
                           size_t p,
                           size_t k) {
        CPPADCG_ASSERT_KNOWN(!to.sparse, "output must be a dense array")
        CPPADCG_ASSERT_KNOWN(to.size >= n, "invalid size")

        Base* values = static_cast<Base*> (to.data);

        if (p == 0) {
            std::copy(&from[0], &from[0] + n, values);
        } else {
            size_t p1 = p + 1;

            for (size_t j = 0; j < n; j++) {
                values[j] += from[j * p1 + k];
            }
        }

    }
};

} // END cg namespace
} // END CppAD namespace

#endif
