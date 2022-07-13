#ifndef CPPAD_CG_FUNCTOR_GENERIC_MODEL_INCLUDED
#define CPPAD_CG_FUNCTOR_GENERIC_MODEL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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

#include <typeinfo>

namespace CppAD {
namespace cg {

/**
 * A model which can be accessed through function pointers.
 * This class is not thread-safe and it should not be used simultaneously in
 * different threads.
 * Multiple instances of this class for the same model from the same model
 * library object can be used simultaneously in different threads.
 *
 * @author Joao Leal
 */
template<class Base>
class FunctorGenericModel : public GenericModel<Base> {
protected:
    static constexpr const char* ERROR_LIBRARY_NOT_READY = "The model library is not ready. The model library that"
                                                           " provided this model might have been closed or deleted.";
protected:
    bool _isLibraryReady;
    /// the model name
    const std::string _name;
    size_t _m;
    size_t _n;
    std::vector<const Base*> _in;
    std::vector<const Base*> _inHess;
    std::vector<Base*> _out;
    LangCAtomicFun _atomicFuncArg;
    std::vector<std::string> _atomicNames; // names of the atomic/external functions required by this model
    std::vector<ExternalFunctionWrapper<Base>* > _atomic;
    size_t _missingAtomicFunctions;
    CppAD::vector<Base> _tx, _ty, _px, _py;
    // original model function
    void (*_zero)(Base const*const*, Base * const*, LangCAtomicFun);
    // first order forward mode
    int (*_forwardOne)(Base const tx[], Base ty[], LangCAtomicFun);
    // first order reverse mode
    int (*_reverseOne)(Base const tx[], Base const ty[], Base px[], Base const py[], LangCAtomicFun);
    // second order reverse mode
    int (*_reverseTwo)(Base const tx[], Base const ty[], Base px[], Base const py[], LangCAtomicFun);
    // jacobian function in the dynamic library
    void (*_jacobian)(Base const*const*, Base * const*, LangCAtomicFun);
    // hessian function in the dynamic library
    void (*_hessian)(Base const*const*, Base * const*, LangCAtomicFun);
    //
    int (*_sparseForwardOne)(unsigned long, Base const *const *, Base * const *, LangCAtomicFun);
    //
    int (*_sparseReverseOne)(unsigned long, Base const *const *, Base * const *, LangCAtomicFun);
    //
    int (*_sparseReverseTwo)(unsigned long, Base const *const *, Base * const *, LangCAtomicFun);
    // sparse jacobian function in the dynamic library
    void (*_sparseJacobian)(Base const*const*, Base * const*, LangCAtomicFun);
    // sparse hessian function in the dynamic library
    void (*_sparseHessian)(Base const*const*, Base * const*, LangCAtomicFun);
    //
    void (*_forwardOneSparsity)(unsigned long, unsigned long const**, unsigned long*);
    //
    void (*_reverseOneSparsity)(unsigned long, unsigned long const**, unsigned long*);
    //
    void (*_reverseTwoSparsity)(unsigned long, unsigned long const**, unsigned long*);
    // jacobian sparsity function in the dynamic library
    void (*_jacobianSparsity)(unsigned long const** row,
            unsigned long const** col,
            unsigned long * nnz);
    // hessian sparsity function in the dynamic library
    void (*_hessianSparsity)(unsigned long const** row,
            unsigned long const** col,
            unsigned long * nnz);
    void (*_hessianSparsity2)(unsigned long i,
            unsigned long const** row,
            unsigned long const** col,
            unsigned long * nnz);
    void (*_atomicFunctions)(const char*** names,
            unsigned long * n);

public:

    FunctorGenericModel(const FunctorGenericModel&) = delete;
    FunctorGenericModel& operator=(const FunctorGenericModel&) = delete;

    virtual ~FunctorGenericModel() {
        for (size_t i = 0; i < _atomic.size(); i++) {
            delete _atomic[i];
        }
    }

    const std::string& getName() const override {
        return _name;
    }

    const std::vector<std::string>& getAtomicFunctionNames() override {
        return _atomicNames;
    }

    bool addAtomicFunction(atomic_base<Base>& atomic) override {
        return addExternalFunction<atomic_base<Base>, AtomicExternalFunctionWrapper<Base> >
                (atomic, atomic.atomic_name());
    }

    bool addExternalModel(GenericModel<Base>& atomic) override {
        return addExternalFunction<GenericModel<Base>, GenericModelExternalFunctionWrapper<Base> >
                (atomic, atomic.getName());
    }

    // Jacobian sparsity
    bool isJacobianSparsityAvailable() override {
        return _jacobianSparsity != nullptr;
    }

    std::vector<bool> JacobianSparsityBool() override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_jacobianSparsity != nullptr, "No Jacobian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_jacobianSparsity)(&row, &col, &nnz);

        bool set_type = true;
        std::vector<bool> s;

        loadSparsity(set_type, s, _m, _n, row, col, nnz);

        return s;
    }

    std::vector<std::set<size_t> > JacobianSparsitySet() override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_jacobianSparsity != nullptr, "No Jacobian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_jacobianSparsity)(&row, &col, &nnz);

        std::set<size_t> set_type;
        std::vector<std::set<size_t> > s;

        loadSparsity(set_type, s, _m, _n, row, col, nnz);

        return s;
    }

    void JacobianSparsity(std::vector<size_t>& equations,
                          std::vector<size_t>& variables) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_jacobianSparsity != nullptr, "No Jacobian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_jacobianSparsity)(&row, &col, &nnz);

        equations.resize(nnz);
        variables.resize(nnz);

        std::copy(row, row + nnz, equations.begin());
        std::copy(col, col + nnz, variables.begin());
    }

    // Hessian sparsity
    bool isHessianSparsityAvailable() override {
        return _hessianSparsity != nullptr;
    }

    std::vector<bool> HessianSparsityBool() override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_hessianSparsity != nullptr, "No Hessian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_hessianSparsity)(&row, &col, &nnz);

        bool set_type = true;
        std::vector<bool> s;

        loadSparsity(set_type, s, _n, _n, row, col, nnz);

        return s;
    }

    std::vector<std::set<size_t> > HessianSparsitySet() override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_hessianSparsity != nullptr, "No Hessian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_hessianSparsity)(&row, &col, &nnz);

        std::set<size_t> set_type;
        std::vector<std::set<size_t> > s;

        loadSparsity(set_type, s, _n, _n, row, col, nnz);

        return s;
    }

    void HessianSparsity(std::vector<size_t>& rows,
                         std::vector<size_t>& cols) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_hessianSparsity != nullptr, "No Hessian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_hessianSparsity)(&row, &col, &nnz);

        rows.resize(nnz);
        cols.resize(nnz);

        std::copy(row, row + nnz, rows.begin());
        std::copy(col, col + nnz, cols.begin());
    }

    bool isEquationHessianSparsityAvailable() override {
        return _hessianSparsity2 != nullptr;
    }

    std::vector<bool> HessianSparsityBool(size_t i) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_hessianSparsity2 != nullptr, "No Hessian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_hessianSparsity2)(i, &row, &col, &nnz);

        bool set_type = true;
        std::vector<bool> s;

        loadSparsity(set_type, s, _n, _n, row, col, nnz);

        return s;
    }

    std::vector<std::set<size_t> > HessianSparsitySet(size_t i) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_hessianSparsity2 != nullptr, "No Hessian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_hessianSparsity2)(i, &row, &col, &nnz);

        std::set<size_t> set_type;
        std::vector<std::set<size_t> > s;

        loadSparsity(set_type, s, _n, _n, row, col, nnz);

        return s;
    }

    void HessianSparsity(size_t i, std::vector<size_t>& rows,
                         std::vector<size_t>& cols) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_hessianSparsity2 != nullptr, "No Hessian sparsity function defined in the dynamic library")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_hessianSparsity2)(i, &row, &col, &nnz);

        rows.resize(nnz);
        cols.resize(nnz);

        std::copy(row, row + nnz, rows.begin());
        std::copy(col, col + nnz, cols.begin());
    }

    /// number of independent variables

    size_t Domain() const override {
        return _n;
    }

    /// number of dependent variables

    size_t Range() const override {
        return _m;
    }

    bool isForwardZeroAvailable() override {
        return _zero != nullptr;
    }

    /// calculate the dependent values (zero order)
    void ForwardZero(ArrayView<const Base> x,
                     ArrayView<Base> dep) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_zero != nullptr, "No zero order forward function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(dep.size() == _m, "Invalid dependent array size")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        _in[0] = x.data();
        _out[0] = dep.data();

        (*_zero)(&_in[0], &_out[0], _atomicFuncArg);
    }

    void ForwardZero(const std::vector<const Base*> &x,
                     ArrayView<Base> dep) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_zero != nullptr, "No zero order forward function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == x.size(), "The number of independent variable arrays is invalid")
        CPPADCG_ASSERT_KNOWN(dep.size() == _m, "Invalid dependent array size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        _out[0] = dep.data();

        (*_zero)(&x[0], &_out[0], _atomicFuncArg);
    }

    void ForwardZero(const CppAD::vector<bool>& vx,
                     CppAD::vector<bool>& vy,
                     ArrayView<const Base> tx,
                     ArrayView<Base> ty) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_zero != nullptr, "No zero order forward function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(tx.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(ty.size() == _m, "Invalid dependent array size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        _in[0] = tx.data();
        _out[0] = ty.data();

        (*_zero)(&_in[0], &_out[0], _atomicFuncArg);

        if (vx.size() > 0) {
            CPPADCG_ASSERT_KNOWN(vx.size() >= _n, "Invalid vx size")
            CPPADCG_ASSERT_KNOWN(vy.size() >= _m, "Invalid vy size")
            const std::vector<std::set<size_t> > jacSparsity = JacobianSparsitySet();
            for (size_t i = 0; i < _m; i++) {
                for (size_t j : jacSparsity[i]) {
                    if (vx[j]) {
                        vy[i] = true;
                        break;
                    }
                }
            }
        }
    }

    bool isJacobianAvailable() override {
        return _jacobian != nullptr;
    }

    /// calculate entire Jacobian
    void Jacobian(ArrayView<const Base> x,
                  ArrayView<Base> jac) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_jacobian != nullptr, "No Jacobian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(jac.size() == _m * _n, "Invalid Jacobian array size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")


        _in[0] = x.data();
        _out[0] = jac.data();

        (*_jacobian)(&_in[0], &_out[0], _atomicFuncArg);
    }

    bool isHessianAvailable() override {
        return _hessian != nullptr;
    }

    /// calculate Hessian for one component of f
    void Hessian(ArrayView<const Base> x,
                 ArrayView<const Base> w,
                 ArrayView<Base> hess) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_hessian != nullptr, "No Hessian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(w.size() == _m, "Invalid multiplier array size")
        CPPADCG_ASSERT_KNOWN(hess.size() == _n * _n, "Invalid Hessian size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        _inHess[0] = x.data();
        _inHess[1] = w.data();
        _out[0] = hess.data();

        (*_hessian)(&_inHess[0], &_out[0], _atomicFuncArg);
    }

    bool isForwardOneAvailable() override {
        return _forwardOne != nullptr;
    }

    void ForwardOne(ArrayView<const Base> tx,
                    ArrayView<Base> ty) override {
        const size_t k = 1;

        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_forwardOne != nullptr, "No forward one function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(tx.size() >= (k + 1) * _n, "Invalid tx size")
        CPPADCG_ASSERT_KNOWN(ty.size() >= (k + 1) * _m, "Invalid ty size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        int ret = (*_forwardOne)(tx.data(), ty.data(), _atomicFuncArg);

        CPPADCG_ASSERT_KNOWN(ret == 0, "First-order forward mode failed.") // generic failure
    }

    bool isSparseForwardOneAvailable() override {
        return _forwardOneSparsity != nullptr && _sparseForwardOne != nullptr;
    }

    void ForwardOne(ArrayView<const Base> x,
                    size_t tx1Nnz, const size_t idx[], const Base tx1[],
                    ArrayView<Base> ty1) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseForwardOne != nullptr, "No sparse forward one function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_forwardOneSparsity != nullptr, "No forward one sparsity function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(x.size() >= _n, "Invalid x size")
        CPPADCG_ASSERT_KNOWN(ty1.size() >= _m, "Invalid ty1 size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        std::fill(ty1.data(), ty1.data() + _m, Base(0));
        if (tx1Nnz == 0)
            return; //nothing to do

        unsigned long const* pos;
        size_t nnz = 0;

        _ty.resize(_m);
        Base* compressed = &_ty[0];

        _inHess[0] = x.data();
        _out[0] = compressed;

        for (size_t ej = 0; ej < tx1Nnz; ej++) {
            size_t j = idx[ej];
            (*_forwardOneSparsity)(j, &pos, &nnz);

            _inHess[1] = &tx1[ej];
            int ret = (*_sparseForwardOne)(j, &_inHess[0], &_out[0], _atomicFuncArg);

            CPPADCG_ASSERT_KNOWN(ret == 0, "First-order forward mode failed.") // generic failure

            for (size_t ePos = 0; ePos < nnz; ePos++) {
                ty1[pos[ePos]] += compressed[ePos];
            }
        }
    }

    bool isReverseOneAvailable() override {
        return _reverseOne != nullptr;
    }

    void ReverseOne(ArrayView<const Base> tx,
                    ArrayView<const Base> ty,
                    ArrayView<Base> px,
                    ArrayView<const Base> py) override {
        const size_t k = 0;
        const size_t k1 = k + 1;

        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_reverseOne != nullptr, "No reverse one function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(tx.size() >= k1 * _n, "Invalid tx size")
        CPPADCG_ASSERT_KNOWN(ty.size() >= k1 * _m, "Invalid ty size")
        CPPADCG_ASSERT_KNOWN(px.size() >= k1 * _n, "Invalid px size")
        CPPADCG_ASSERT_KNOWN(py.size() >= k1 * _m, "Invalid py size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        int ret = (*_reverseOne)(tx.data(), ty.data(), px.data(), py.data(), _atomicFuncArg);

        CPPADCG_ASSERT_KNOWN(ret == 0, "First-order reverse mode failed.")
    }

    bool isSparseReverseOneAvailable() override {
        return _reverseOneSparsity != nullptr && _sparseReverseOne != nullptr;
    }

    void ReverseOne(ArrayView<const Base> x,
                    ArrayView<Base> px,
                    size_t pyNnz, const size_t idx[], const Base py[]) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseReverseOne != nullptr, "No sparse reverse one function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_reverseOneSparsity != nullptr, "No reverse one sparsity function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(x.size() >= _n, "Invalid x size")
        CPPADCG_ASSERT_KNOWN(px.size() >= _n, "Invalid px size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        std::fill(px.data(), px.data() + _n, Base(0));
        if (pyNnz == 0)
            return; //nothing to do

        unsigned long const* pos;
        size_t nnz = 0;

        _px.resize(_n);
        Base* compressed = &_px[0];

        _inHess[0] = x.data();
        _out[0] = compressed;

        for (size_t ei = 0; ei < pyNnz; ei++) {
            size_t i = idx[ei];
            (*_reverseOneSparsity)(i, &pos, &nnz);

            _inHess[1] = &py[ei];
            int ret = (*_sparseReverseOne)(i, &_inHess[0], &_out[0], _atomicFuncArg);

            CPPADCG_ASSERT_KNOWN(ret == 0, "First-order reverse mode failed.")

            for (size_t ePos = 0; ePos < nnz; ePos++) {
                px[pos[ePos]] += compressed[ePos];
            }
        }
    }

    bool isReverseTwoAvailable() override {
        return _reverseTwo != nullptr;
    }

    void ReverseTwo(ArrayView<const Base> tx,
                    ArrayView<const Base> ty,
                    ArrayView<Base> px,
                    ArrayView<const Base> py) override {
        const size_t k = 1;
        const size_t k1 = k + 1;

        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_reverseTwo != nullptr, "No sparse reverse two function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1")
        CPPADCG_ASSERT_KNOWN(tx.size() >= k1 * _n, "Invalid tx size")
        CPPADCG_ASSERT_KNOWN(ty.size() >= k1 * _m, "Invalid ty size")
        CPPADCG_ASSERT_KNOWN(px.size() >= k1 * _n, "Invalid px size")
        CPPADCG_ASSERT_KNOWN(py.size() >= k1 * _m, "Invalid py size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        int ret = (*_reverseTwo)(tx.data(), ty.data(), px.data(), py.data(), _atomicFuncArg);

        CPPADCG_ASSERT_KNOWN(ret != 1, "Second-order reverse mode failed: py[2*i] (i=0...m) must be zero.")
        CPPADCG_ASSERT_KNOWN(ret == 0, "Second-order reverse mode failed.")
    }

    bool isSparseReverseTwoAvailable() override {
        return _sparseReverseTwo != nullptr;
    }

    void ReverseTwo(ArrayView<const Base> x,
                    size_t tx1Nnz, const size_t idx[], const Base tx1[],
                    ArrayView<Base> px2,
                    ArrayView<const Base> py2) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseReverseTwo != nullptr, "No sparse reverse two function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_reverseTwoSparsity != nullptr, "No reverse two sparsity function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(x.size() >= _n, "Invalid x size")
        CPPADCG_ASSERT_KNOWN(px2.size() >= _n, "Invalid px2 size")
        CPPADCG_ASSERT_KNOWN(py2.size() >= _m, "Invalid py2 size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        std::fill(px2.data(), px2.data() + _n, Base(0));
        if (tx1Nnz == 0)
            return; //nothing to do

        unsigned long const* pos;
        size_t nnz = 0;

        _px.resize(_n);
        Base* compressed = &_px[0];

        const Base * in[3];
        in[0] = x.data();
        in[2] = py2.data();
        _out[0] = compressed;

        for (size_t ej = 0; ej < tx1Nnz; ej++) {
            size_t j = idx[ej];
            (*_reverseTwoSparsity)(j, &pos, &nnz);

            in[1] = &tx1[ej];
            int ret = (*_sparseReverseTwo)(j, &in[0], &_out[0], _atomicFuncArg);

            CPPADCG_ASSERT_KNOWN(ret == 0, "Second-order reverse mode failed.") // generic failure

            for (size_t ePos = 0; ePos < nnz; ePos++) {
                px2[pos[ePos]] += compressed[ePos];
            }
        }
    }

    bool isSparseJacobianAvailable() override {
        return _jacobianSparsity != nullptr && _sparseJacobian != nullptr;
    }

    /// calculate sparse Jacobians

    void SparseJacobian(ArrayView<const Base> x,
                        ArrayView<Base> jac) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseJacobian != nullptr, "No sparse jacobian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(jac.size() == _m * _n, "Invalid Jacobian size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* row;
        unsigned long const* col;
        unsigned long nnz;
        (*_jacobianSparsity)(&row, &col, &nnz);

        CppAD::vector<Base> compressed(nnz);

        if (nnz > 0) {
            _in[0] = x.data();
            _out[0] = &compressed[0];

            (*_sparseJacobian)(&_in[0], &_out[0], _atomicFuncArg);
        }

        createDenseFromSparse(compressed,
                              _m, _n,
                              row, col,
                              nnz,
                              jac);
    }

    void SparseJacobian(const std::vector<Base> &x,
                        std::vector<Base>& jac,
                        std::vector<size_t>& row,
                        std::vector<size_t>& col) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseJacobian != nullptr, "No sparse Jacobian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* drow;
        unsigned long const* dcol;
        unsigned long nnz;
        (*_jacobianSparsity)(&drow, &dcol, &nnz);

        jac.resize(nnz);
        row.resize(nnz);
        col.resize(nnz);

        if (nnz > 0) {
            _in[0] = &x[0];
            _out[0] = &jac[0];

            (*_sparseJacobian)(&_in[0], &_out[0], _atomicFuncArg);
            std::copy(drow, drow + nnz, row.begin());
            std::copy(dcol, dcol + nnz, col.begin());
        }
    }

    void SparseJacobian(ArrayView<const Base> x,
                        ArrayView<Base> jac,
                        size_t const** row,
                        size_t const** col) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseJacobian != nullptr, "No sparse Jacobian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* drow;
        unsigned long const* dcol;
        unsigned long nnz;
        (*_jacobianSparsity)(&drow, &dcol, &nnz);
        CPPADCG_ASSERT_KNOWN(nnz == jac.size(), "Invalid number of non-zero elements in Jacobian")
        *row = drow;
        *col = dcol;

        if (nnz > 0) {
            _in[0] = x.data();
            _out[0] = jac.data();

            (*_sparseJacobian)(&_in[0], &_out[0], _atomicFuncArg);
        }
    }

    void SparseJacobian(const std::vector<const Base*>& x,
                        ArrayView<Base> jac,
                        size_t const** row,
                        size_t const** col) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseJacobian != nullptr, "No sparse Jacobian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == x.size(), "The number of independent variable arrays is invalid")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* drow;
        unsigned long const* dcol;
        unsigned long nnz;
        (*_jacobianSparsity)(&drow, &dcol, &nnz);
        CPPADCG_ASSERT_KNOWN(nnz == jac.size(), "Invalid number of non-zero elements in Jacobian")
        *row = drow;
        *col = dcol;

        if (nnz > 0) {
            _out[0] = jac.data();

            (*_sparseJacobian)(&x[0], &_out[0], _atomicFuncArg);
        }
    }

    bool isSparseHessianAvailable() override {
        return _hessianSparsity != nullptr && _sparseHessian != nullptr;
    }

    /// calculate sparse Hessians

    void SparseHessian(ArrayView<const Base> x,
                       ArrayView<const Base> w,
                       ArrayView<Base> hess) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseHessian != nullptr, "No sparse Hessian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(w.size() == _m, "Invalid multiplier array size")
        // CPPADCG_ASSERT_KNOWN(hess.size() == _n * _n, "Invalid Hessian size")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* row, *col;
        unsigned long nnz;
        (*_hessianSparsity)(&row, &col, &nnz);

        CppAD::vector<Base> compressed(nnz);
        if (nnz > 0) {
            _inHess[0] = x.data();
            _inHess[1] = w.data();
            _out[0] = &compressed[0];

            (*_sparseHessian)(&_inHess[0], &_out[0], _atomicFuncArg);
        }

        createDenseFromSparse(compressed,
                              _n, _n,
                              row, col,
                              nnz,
                              hess);
    }

    void SparseHessian(const std::vector<Base> &x,
                       const std::vector<Base> &w,
                       std::vector<Base>& hess,
                       std::vector<size_t>& row,
                       std::vector<size_t>& col) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseHessian != nullptr, "No sparse Hessian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(w.size() == _m, "Invalid multiplier array size")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* drow, *dcol;
        unsigned long nnz;
        (*_hessianSparsity)(&drow, &dcol, &nnz);

        hess.resize(nnz);
        row.resize(nnz);
        col.resize(nnz);

        if (nnz > 0) {
            std::copy(drow, drow + nnz, row.begin());
            std::copy(dcol, dcol + nnz, col.begin());

            _inHess[0] = &x[0];
            _inHess[1] = &w[0];
            _out[0] = &hess[0];

            (*_sparseHessian)(&_inHess[0], &_out[0], _atomicFuncArg);
        }
    }

    void SparseHessian(ArrayView<const Base> x,
                       ArrayView<const Base> w,
                       ArrayView<Base> hess,
                       size_t const** row,
                       size_t const** col) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseHessian != nullptr, "No sparse Hessian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == 1, "The number of independent variable arrays is higher than 1,"
                             " please use the variable size methods")
        CPPADCG_ASSERT_KNOWN(x.size() == _n, "Invalid independent array size")
        CPPADCG_ASSERT_KNOWN(w.size() == _m, "Invalid multiplier array size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* drow, *dcol;
        unsigned long nnz;
        (*_hessianSparsity)(&drow, &dcol, &nnz);
        CPPADCG_ASSERT_KNOWN(nnz == hess.size(), "Invalid number of non-zero elements in Hessian")
        *row = drow;
        *col = dcol;

        if (nnz > 0) {
            _inHess[0] = x.data();
            _inHess[1] = w.data();
            _out[0] = hess.data();

            (*_sparseHessian)(&_inHess[0], &_out[0], _atomicFuncArg);
        }
    }

    void SparseHessian(const std::vector<const Base*>& x,
                       ArrayView<const Base> w,
                       ArrayView<Base> hess,
                       size_t const** row,
                       size_t const** col) override {
        CPPADCG_ASSERT_KNOWN(_isLibraryReady, ERROR_LIBRARY_NOT_READY)
        CPPADCG_ASSERT_KNOWN(_sparseHessian != nullptr, "No sparse Hessian function defined in the dynamic library")
        CPPADCG_ASSERT_KNOWN(_in.size() == x.size(), "The number of independent variable arrays is invalid")
        CPPADCG_ASSERT_KNOWN(w.size() == _m, "Invalid multiplier array size")
        CPPADCG_ASSERT_KNOWN(_missingAtomicFunctions == 0, "Some atomic functions used by the compiled model have not been specified yet")

        unsigned long const* drow, *dcol;
        unsigned long nnz;
        (*_hessianSparsity)(&drow, &dcol, &nnz);
        CPPADCG_ASSERT_KNOWN(nnz == hess.size(), "Invalid number of non-zero elements in Hessian")
        *row = drow;
        *col = dcol;

        if (nnz > 0) {
            std::copy(x.begin(), x.end(), _inHess.begin());
            _inHess.back() = w.data(); // the index might not be 1
            _out[0] = hess.data();

            (*_sparseHessian)(&_inHess[0], &_out[0], _atomicFuncArg);
        }
    }

protected:

    /**
     * Creates a new model
     *
     * @param name The model name
     */
    explicit FunctorGenericModel(std::string name) :
        _isLibraryReady(false),
        _name(std::move(name)),
        _m(0),
        _n(0),
        _atomicFuncArg{nullptr}, // not really required
        _missingAtomicFunctions(0),
        _zero(nullptr),
        _forwardOne(nullptr),
        _reverseOne(nullptr),
        _reverseTwo(nullptr),
        _jacobian(nullptr),
        _hessian(nullptr),
        _sparseForwardOne(nullptr),
        _sparseReverseOne(nullptr),
        _sparseReverseTwo(nullptr),
        _sparseJacobian(nullptr),
        _sparseHessian(nullptr),
        _forwardOneSparsity(nullptr),
        _reverseOneSparsity(nullptr),
        _reverseTwoSparsity(nullptr),
        _jacobianSparsity(nullptr),
        _hessianSparsity(nullptr),
        _hessianSparsity2(nullptr),
        _atomicFunctions(nullptr) {

    }

    virtual void init() {
        // validate the dynamic library
        validate();

        // load functions from the dynamic library
        loadFunctions();
    }

    virtual void* loadFunction(const std::string& functionName,
                               bool required = true) = 0;

    virtual void validate() {
        /**
         * Check the data type
         */
        void (*infoFunc)(const char** baseName, unsigned long*, unsigned long*, unsigned int*, unsigned int*);
        infoFunc = reinterpret_cast<decltype(infoFunc)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_INFO));

        // local
        const char* localBaseName = typeid (Base).name();
        std::string local = ModelCSourceGen<Base>::baseTypeName() + "  " + localBaseName;

        // from dynamic library
        const char* dynamicLibBaseName = nullptr;
        unsigned int inSize = 0;
        unsigned int outSize = 0;
        (*infoFunc)(&dynamicLibBaseName, &_m, &_n, &inSize, &outSize);

        _in.resize(inSize);
        _inHess.resize(inSize + 1);
        _out.resize(outSize);

        CPPADCG_ASSERT_KNOWN(local == std::string(dynamicLibBaseName),
                             (std::string("Invalid data type in dynamic library. Expected '") + local
                             + "' but the library provided '" + dynamicLibBaseName + "'.").c_str())
        CPPADCG_ASSERT_KNOWN(inSize > 0,
                             "Invalid dimension received from the dynamic library.")
        CPPADCG_ASSERT_KNOWN(outSize > 0,
                             "Invalid dimension received from the dynamic library.")

        _isLibraryReady = true;
    }

    virtual void loadFunctions() {
        _zero = reinterpret_cast<decltype(_zero)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_FORWAD_ZERO, false));
        _forwardOne = reinterpret_cast<decltype(_forwardOne)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_FORWARD_ONE, false));
        _reverseOne = reinterpret_cast<decltype(_reverseOne)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_REVERSE_ONE, false));
        _reverseTwo = reinterpret_cast<decltype(_reverseTwo)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_REVERSE_TWO, false));
        _jacobian = reinterpret_cast<decltype(_jacobian)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_JACOBIAN, false));
        _hessian = reinterpret_cast<decltype(_hessian)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_HESSIAN, false));
        _sparseForwardOne = reinterpret_cast<decltype(_sparseForwardOne)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_SPARSE_FORWARD_ONE, false));
        _sparseReverseOne = reinterpret_cast<decltype(_sparseReverseOne)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_SPARSE_REVERSE_ONE, false));
        _sparseReverseTwo = reinterpret_cast<decltype(_sparseReverseTwo)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_SPARSE_REVERSE_TWO, false));
        _sparseJacobian = reinterpret_cast<decltype(_sparseJacobian)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_SPARSE_JACOBIAN, false));
        _sparseHessian = reinterpret_cast<decltype(_sparseHessian)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_SPARSE_HESSIAN, false));
        _forwardOneSparsity = reinterpret_cast<decltype(_forwardOneSparsity)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_FORWARD_ONE_SPARSITY, false));
        _reverseOneSparsity = reinterpret_cast<decltype(_reverseOneSparsity)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_REVERSE_ONE_SPARSITY, false));
        _reverseTwoSparsity = reinterpret_cast<decltype(_reverseTwoSparsity)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_REVERSE_TWO_SPARSITY, false));
        _jacobianSparsity = reinterpret_cast<decltype(_jacobianSparsity)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_JACOBIAN_SPARSITY, false));
        _hessianSparsity = reinterpret_cast<decltype(_hessianSparsity)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_HESSIAN_SPARSITY, false));
        _hessianSparsity2 = reinterpret_cast<decltype(_hessianSparsity2)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_HESSIAN_SPARSITY2, false));
        _atomicFunctions = reinterpret_cast<decltype(_atomicFunctions)>(loadFunction(_name + "_" + ModelCSourceGen<Base>::FUNCTION_ATOMIC_FUNC_NAMES, true));

        CPPADCG_ASSERT_KNOWN((_sparseForwardOne == nullptr) == (_forwardOneSparsity == nullptr), "Missing functions in the dynamic library")
        CPPADCG_ASSERT_KNOWN((_sparseForwardOne == nullptr) == (_forwardOne == nullptr), "Missing functions in the dynamic library")
        CPPADCG_ASSERT_KNOWN((_sparseReverseOne == nullptr) == (_reverseOneSparsity == nullptr), "Missing functions in the dynamic library")
        CPPADCG_ASSERT_KNOWN((_sparseReverseOne == nullptr) == (_reverseOne == nullptr), "Missing functions in the dynamic library")
        CPPADCG_ASSERT_KNOWN((_sparseReverseTwo == nullptr) == (_reverseTwoSparsity == nullptr), "Missing functions in the dynamic library")
        CPPADCG_ASSERT_KNOWN((_sparseReverseTwo == nullptr) == (_reverseTwo == nullptr), "Missing functions in the dynamic library")
        CPPADCG_ASSERT_KNOWN((_sparseJacobian == nullptr) || (_jacobianSparsity != nullptr), "Missing functions in the dynamic library")
        CPPADCG_ASSERT_KNOWN((_sparseHessian == nullptr) || (_hessianSparsity != nullptr), "Missing functions in the dynamic library")

        /**
         * Prepare the atomic functions argument
         */
        const char** names;
        unsigned long n;
        (*_atomicFunctions)(&names, &n);
        _atomic.resize(n);
        _atomicNames.resize(n);
        for (unsigned long i = 0; i < n; ++i) {
            _atomicNames[i] = std::string(names[i]);
        }

        _atomicFuncArg.libModel = this;
        _atomicFuncArg.forward = &atomicForward;
        _atomicFuncArg.reverse = &atomicReverse;

        _missingAtomicFunctions = n;
    }

    template <class VectorSet>
    inline void loadSparsity(bool set_type,
                             VectorSet& s,
                             unsigned long nrows, unsigned long ncols,
                             unsigned long const* rows, unsigned long const* cols,
                             unsigned long nnz) {
        s.resize(nrows * ncols, false);

        for (unsigned long i = 0; i < nnz; i++) {
            s[rows[i] * ncols + cols[i]] = true;
        }
    }

    template <class VectorSet>
    inline void loadSparsity(const std::set<size_t>& set_type,
                             VectorSet& s,
                             unsigned long nrows, unsigned long ncols,
                             unsigned long const* rows, unsigned long const* cols,
                             unsigned long nnz) {

        // dimension size of result vector
        s.resize(nrows);

        for (unsigned long i = 0; i < nnz; i++) {
            s[rows[i]].insert(cols[i]);
        }
    }

    inline void createDenseFromSparse(const CppAD::vector<Base>& compressed,
                                      unsigned long nrows, unsigned long ncols,
                                      unsigned long const* rows, unsigned long const* cols,
                                      unsigned long nnz,
                                      ArrayView<Base> mat) const {
        CPPADCG_ASSERT_KNOWN(mat.size() == nrows * ncols, "Invalid matrix size")
        mat.fill(Base(0));

        for (size_t i = 0; i < nnz; i++) {
            mat[rows[i] * ncols + cols[i]] = compressed[i];
        }
    }

    virtual void modelLibraryClosed() {
        _isLibraryReady = false;
        _zero = nullptr;
        _forwardOne = nullptr;
        _reverseOne = nullptr;
        _reverseTwo = nullptr;
        _jacobian = nullptr;
        _hessian = nullptr;
        _sparseForwardOne = nullptr;
        _sparseReverseOne = nullptr;
        _sparseReverseTwo = nullptr;
        _sparseJacobian = nullptr;
        _sparseHessian = nullptr;
        _forwardOneSparsity = nullptr;
        _reverseOneSparsity = nullptr;
        _reverseTwoSparsity = nullptr;
        _jacobianSparsity = nullptr;
        _hessianSparsity = nullptr;
        _hessianSparsity2 = nullptr;
    }

private:

    template<class ExtFunc, class Wrapper>
    inline bool addExternalFunction(ExtFunc& atomic,
                                    const std::string& name) {
        size_t n = _atomicNames.size();
        for (size_t i = 0; i < n; i++) {
            if (name == _atomicNames[i]) {
                if (_atomic[i] == nullptr) {
                    _missingAtomicFunctions--;
                } else {
                    delete _atomic[i];
                }
                _atomic[i] = new Wrapper(atomic);
                return true;
            }
        }
        return false;
    }

    static int atomicForward(void* libModelIn,
                             int atomicIndex,
                             int q,
                             int p,
                             const Array tx[],
                             Array* ty) {
        auto* libModel = static_cast<FunctorGenericModel<Base>*> (libModelIn);
        ExternalFunctionWrapper<Base>* externalFunc = libModel->_atomic[atomicIndex];

        return externalFunc->forward(*libModel, q, p, tx, *ty);
    }

    static int atomicReverse(void* libModelIn,
                             int atomicIndex,
                             int p,
                             const Array tx[],
                             Array* px,
                             const Array py[]) {
        auto* libModel = static_cast<FunctorGenericModel<Base>*> (libModelIn);
        ExternalFunctionWrapper<Base>* externalFunc = libModel->_atomic[atomicIndex];

        return externalFunc->reverse(*libModel, p, tx, *px, py);
    }
#ifdef CPPAD_CG_SYSTEM_LINUX
    friend class LinuxDynamicLib<Base>;
#endif
    friend class AtomicExternalFunctionWrapper<Base>;
};

} // END cg namespace
} // END CppAD namespace

#endif
