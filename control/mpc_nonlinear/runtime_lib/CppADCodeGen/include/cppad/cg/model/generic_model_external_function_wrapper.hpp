#ifndef CPPAD_CG_GENERIC_MODEL_EXTERNAL_FUNCTION_WRAPPER_INCLUDED
#define CPPAD_CG_GENERIC_MODEL_EXTERNAL_FUNCTION_WRAPPER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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
class GenericModelExternalFunctionWrapper : public ExternalFunctionWrapper<Base> {
private:
    GenericModel<Base>* model_;
public:

    inline GenericModelExternalFunctionWrapper(GenericModel<Base>& model) :
        model_(&model) {
    }

    inline virtual ~GenericModelExternalFunctionWrapper() {
    }

    virtual bool forward(FunctorGenericModel<Base>& libModel,
                         int q,
                         int p,
                         const Array tx[],
                         Array& ty) {
        CPPADCG_ASSERT_KNOWN(!tx[0].sparse, "independent array must be dense");
        ArrayView<const Base> x(static_cast<const Base*> (tx[0].data), tx[0].size);

        CPPADCG_ASSERT_KNOWN(!ty.sparse, "dependent array must be dense");
        ArrayView<Base> y(static_cast<Base*> (ty.data), ty.size);


        if (p == 0) {
            model_->ForwardZero(x, y);
            return true;

        } else if (p == 1) {
            CPPADCG_ASSERT_KNOWN(tx[1].sparse, "independent Taylor array must be sparse");
            Base* tx1 = static_cast<Base*> (tx[1].data);

            model_->ForwardOne(x,
                               tx[1].nnz, tx[1].idx, tx1,
                               y);
            return true;
        }

        return false;
    }

    virtual bool reverse(FunctorGenericModel<Base>& libModel,
                         int p,
                         const Array tx[],
                         Array& px,
                         const Array py[]) {
        CPPADCG_ASSERT_KNOWN(!tx[0].sparse, "independent array must be dense");
        ArrayView<const Base> x(static_cast<const Base*> (tx[0].data), tx[0].size);

        CPPADCG_ASSERT_KNOWN(!px.sparse, "independent partials array must be dense");
        ArrayView<Base> pxb(static_cast<Base*> (px.data), px.size);

        if (p == 0) {
            CPPADCG_ASSERT_KNOWN(py[0].sparse, "dependent partials array must be sparse");
            Base* pyb = static_cast<Base*> (py[0].data);

            model_->ReverseOne(x,
                               pxb,
                               py[0].nnz, py[0].idx, pyb);
            return true;

        } else if (p == 1) {
            CPPADCG_ASSERT_KNOWN(tx[1].sparse, "independent array must be sparse");
            const Base* tx1 = static_cast<const Base*> (tx[1].data);
            CPPADCG_ASSERT_KNOWN(py[0].sparse, "dependent partials array must be sparse");
            CPPADCG_ASSERT_KNOWN(py[0].nnz == 0, "first order dependent partials must be zero");
            CPPADCG_ASSERT_KNOWN(!py[1].sparse, "independent partials array must be dense");
            ArrayView<const Base> py2(static_cast<Base*> (py[1].data), py[1].size);

            model_->ReverseTwo(x,
                               tx[1].nnz, tx[1].idx, tx1,
                               pxb,
                               py2);
            return true;
        }

        return false;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
