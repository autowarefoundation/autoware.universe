#ifndef CPPAD_CG_GENERIC_MODEL_INCLUDED
#define CPPAD_CG_GENERIC_MODEL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *    Copyright (C) 2020 Joao Leal
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
 * Abstract class used to execute a generated model
 * 
 * @author Joao Leal
 */
template<class Base>
class GenericModel {
protected:
    CGAtomicGenericModel<Base>* _atomic;
    // whether or not to evaluate forward mode of atomics during a reverse sweep
    bool _evalAtomicForwardOne4CppAD;
public:

    GenericModel() :
        _atomic(nullptr),
        _evalAtomicForwardOne4CppAD(true) {
    }

    inline virtual ~GenericModel() {
        delete _atomic;
    }

    /**
     * Provides the name for this model.
     * 
     * @return The model name
     */
    virtual const std::string& getName() const = 0;


    /**
     * Determines whether or not the Jacobian sparsity pattern can be requested.
     *
     * @return true if it is possible to request the Jacobian sparsity pattern
     */
    virtual bool isJacobianSparsityAvailable() = 0;

    // Jacobian sparsity
    virtual std::vector<std::set<size_t> > JacobianSparsitySet() = 0;
    virtual std::vector<bool> JacobianSparsityBool() = 0;
    virtual void JacobianSparsity(std::vector<size_t>& equations,
                                  std::vector<size_t>& variables) = 0;

    /**
     * Determines whether or not the sparsity pattern for the weighted sum of
     * the Hessians can be requested.
     *
     * @return true if it is possible to request the parsity pattern for the
     *         weighted sum of the Hessians
     */
    virtual bool isHessianSparsityAvailable() = 0;

    /**
     * Provides the sparsity of the sum of the hessian for each dependent 
     * variable.
     * 
     * @return The sparsity
     */
    virtual std::vector<std::set<size_t> > HessianSparsitySet() = 0;
    virtual std::vector<bool> HessianSparsityBool() = 0;
    virtual void HessianSparsity(std::vector<size_t>& rows,
                                 std::vector<size_t>& cols) = 0;

    /**
     * Determines whether or not the sparsity pattern for the Hessian
     * associated with a dependent variable can be requested.
     *
     * @return true if it is possible to request the parsity pattern for the
     *         Hessians
     */
    virtual bool isEquationHessianSparsityAvailable() = 0;

    /**
     * Provides the sparsity of the hessian for a dependent variable
     * 
     * @param i The index of the dependent variable
     * @return The sparsity
     */
    virtual std::vector<std::set<size_t> > HessianSparsitySet(size_t i) = 0;
    virtual std::vector<bool> HessianSparsityBool(size_t i) = 0;
    virtual void HessianSparsity(size_t i,
                                 std::vector<size_t>& rows,
                                 std::vector<size_t>& cols) = 0;

    /**
     * Provides the number of independent variables.
     * 
     * @return The number of independent variables
     */
    virtual size_t Domain() const = 0;

    /**
     * Provides the number of dependent variables.
     * 
     * @return The number of dependent variables.
     */
    virtual size_t Range() const = 0;

    /**
     * The names of the atomic functions required by this model.
     * All external/atomic functions must be provided before using
     * this model to compute numerical values.
     *
     * @return the names of the atomic functions used by this model
     */
    virtual const std::vector<std::string>& getAtomicFunctionNames() = 0;

    /**
     * Defines a CppAD atomic function to be used as an external function 
     * by the compiled code.
     * It should match an external function name previously provided to
     * create the source.
     * 
     * @param atomic The atomic function. This object must only be deleted
     *               after the model.
     * @return true if the atomic function is required by the model, false
     *         if it will never be used.
     */
    virtual bool addAtomicFunction(atomic_base<Base>& atomic) = 0;

    /**
     * Defines a generic model to be used as an external function by the
     * compiled code.
     * It should match an external function name previously provided to
     * create the source. This form should be preferred over 
     * ::addAtomicFunction whenever possible.
     * 
     * @param atomic The generic model. This object must only be deleted
     *               after the model.
     * @return true if the external function is required by the model, false
     *         if it will never be used.
     */
    virtual bool addExternalModel(GenericModel<Base>& atomic) = 0;

    /**
     * Defines whether or not to evaluate a forward mode of an atomic 
     * functions during a reverse sweep so that CppAD checks validate OK.
     * If this model is not used within CppAD then it should be set to false.
     * 
     * @param evalForwardOne4CppAD true to perform the forward mode, 
     *                             false to ignore it
     */
    inline void setAtomicEvalForwardOne4CppAD(bool evalForwardOne4CppAD) {
        _evalAtomicForwardOne4CppAD = evalForwardOne4CppAD;
    }

    inline bool isAtomicEvalForwardOne4CppAD() const {
        return _evalAtomicForwardOne4CppAD;
    }

    /***********************************************************************
     *                        Forward zero
     **********************************************************************/

    /**
     * Determines whether or not the model evaluation (zero-order forward mode)
     * can be requested.
     *
     * @return true if it is possible to evaluate the model
     */
    virtual bool isForwardZeroAvailable() = 0;

    /**
     * Evaluates the dependent model variables (zero-order).
     * This method considers that the generic model was prepared
     * with a single array for the independent variables (the default
     * behavior).
     * 
     * @param x The independent variable vector
     * @return The dependent variable vector
     */
    template<typename VectorBase>
    inline VectorBase ForwardZero(const VectorBase& x) {
        VectorBase dep(Range());
        this->ForwardZero(ArrayView<const Base>(&x[0], x.size()),
                          ArrayView<Base>(&dep[0], dep.size()));
        return dep;
    }

    virtual void ForwardZero(const CppAD::vector<bool>& vx,
                             CppAD::vector<bool>& vy,
                             ArrayView<const Base> tx,
                             ArrayView<Base> ty) = 0;

    /**
     * Evaluates the dependent model variables (zero-order).
     * This method considers that the generic model was prepared
     * using a single array for the independent variables (the default
     * behavior).
     * 
     * @param x The independent variable vector
     * @param dep The dependent variable vector
     */
    template<typename VectorBase>
    inline void ForwardZero(const VectorBase& x,
                            VectorBase& dep) {
        dep.resize(Range());
        this->ForwardZero(ArrayView<const Base>(&x[0], x.size()),
                          ArrayView<Base>(&dep[0], dep.size()));
    }

    virtual void ForwardZero(ArrayView<const Base> x,
                             ArrayView<Base> dep) = 0;

    /**
     * Determines the dependent variable values using a variable number of 
     * independent variable arrays.
     * This method can be useful if the generic model was prepared
     * considering that the independent variables are provided by several
     * arrays.
     * 
     * @param x Contains the several independent variable vectors
     * @param dep The values of the dependent variables
     */
    virtual void ForwardZero(const std::vector<const Base*> &x,
                             ArrayView<Base> dep) = 0;

    /***********************************************************************
     *                        Dense Jacobian
     **********************************************************************/

    /**
     * Determines whether or not the dense Jacobian evaluation can be
     * requested.
     *
     * @return true if it is possible to evaluate the dense Jacobian
     */
    virtual bool isJacobianAvailable() = 0;

    template<typename VectorBase>
    inline VectorBase Jacobian(const VectorBase& x) {
        VectorBase jac(Range() * Domain());
        Jacobian(ArrayView<const Base>(&x[0], x.size()),
                 ArrayView<Base>(&jac[0], jac.size()));
        return jac;
    }

    template<typename VectorBase>
    inline void Jacobian(const VectorBase& x,
                         VectorBase& jac) {
        jac.resize(Range() * Domain());
        Jacobian(ArrayView<const Base>(&x[0], x.size()),
                 ArrayView<Base>(&jac[0], jac.size()));
    }

    virtual void Jacobian(ArrayView<const Base> x,
                          ArrayView<Base> jac) = 0;

    /***********************************************************************
     *                        Dense Hessian
     **********************************************************************/

    /**
     * Determines whether or not the dense evaluation of the weigthed sum of
     * the Hessians can be requested.
     *
     * @return true if it is possible to evaluate the dense weigthed sum of
     *         the Hessians
     */
    virtual bool isHessianAvailable() = 0;

    template<typename VectorBase>
    inline VectorBase Hessian(const VectorBase& x,
                              const VectorBase& w) {
        VectorBase hess(Domain() * Domain());
        this->Hessian(ArrayView<const Base>(&x[0], x.size()),
                      ArrayView<const Base>(&w[0], w.size()),
                      ArrayView<Base>(&hess[0], hess.size()));
        return hess;
    }

    template<typename VectorBase>
    inline void Hessian(const VectorBase& x,
                        const VectorBase& w,
                        VectorBase& hess) {
        hess.resize(Domain() * Domain());
        this->Hessian(ArrayView<const Base>(&x[0], x.size()),
                      ArrayView<const Base>(&w[0], w.size()),
                      ArrayView<Base>(&hess[0], hess.size()));
    }

    /// calculate Hessian for one component of f

    template<typename VectorBase>
    inline VectorBase Hessian(const VectorBase& x,
                              size_t i) {
        CPPADCG_ASSERT_KNOWN(i < Range(), "Invalid equation index")

        VectorBase w(Range());
        w[i] = 1.0;
        VectorBase hess(Domain() * Domain());
        this->Hessian(ArrayView<const Base>(&x[0], x.size()),
                      ArrayView<const Base>(&w[0], w.size()),
                      ArrayView<Base>(&hess[0], hess.size()));
        return hess;
    }

    virtual void Hessian(ArrayView<const Base> x,
                         ArrayView<const Base> w,
                         ArrayView<Base> hess) = 0;

    /***********************************************************************
     *                        Forward one
     **********************************************************************/

    /**
     * Determines whether or not the first-order forward mode dense
     * methods can be called.
     *
     * @return true if it is possible to evaluate the first-order forward mode
     *         using the dense vector format
     */
    virtual bool isForwardOneAvailable() = 0;

    /**
     * Computes results during a forward mode sweep. 
     * Computes the first-order Taylor coefficients for dependent variables
     * relative to a single independent variable.
     * This method can be used during the evaluation of the jacobian when
     * the model is used through a user defined external/atomic AD function.
     * @warning do not used it as a generic forward mode function!
     * 
     * @param tx The Taylor coefficients of the independent variables 
     * @return The Taylor coefficients of the dependent variables 
     */
    template<typename VectorBase>
    inline VectorBase ForwardOne(const VectorBase& tx) {
        size_t m = Range();
        const size_t k = 1;
        VectorBase ty((k + 1) * m);

        this->ForwardOne(ArrayView<const Base>(&tx[0], tx.size()),
                         ArrayView<Base>(&ty[0], ty.size()));

        VectorBase dy(m);
        for (size_t i = 0; i < m; i++) {
            dy[i] = ty[i * (k + 1) + k];
        }

        return dy;
    }

    /**
     * Computes results during a forward mode sweep. 
     * Computes the first-order Taylor coefficients for dependent variables
     * relative to a single independent variable.
     * This method can be used during the evaluation of the jacobian when
     * the model is used through a user defined external/atomic AD function.
     * @warning do not used it as a generic forward mode function!
     * 
     * @param tx The Taylor coefficients of the independent variables 
     * @param ty The Taylor coefficients of the dependent variables 
     */
    virtual void ForwardOne(ArrayView<const Base> tx,
                            ArrayView<Base> ty) = 0;

    /**
     * Determines whether or not the first-order forward mode sparse
     * method can be called.
     *
     * @return true if it is possible to evaluate the first-order forward mode
     *         using the sparse vector format
     */
    virtual bool isSparseForwardOneAvailable() = 0;

    /**
     * Computes results during a first-order forward mode sweep, the
     * first-order Taylor coefficients for dependent variables relative to
     * a single independent variable.
     * This method can be used during the evaluation of the jacobian when
     * the model is used through a user defined external/atomic AD function.
     * This method version avoids some data copies and can be more efficient.
     * @warning do not used it as a generic forward mode function!
     * 
     * @param x independent variable vector
     * @param tx1Nnz the number of non-zeros of the directional derivatives
     *               of the independent variables (seed directions)
     * @param idx the locations of the non-zero values the partial 
     *            derivatives of the dependent variables (seeds)
     * @param tx1 the non-zero values of the partial derivatives of the 
     *           dependent variables (seeds)
     * @param ty1
     */
    virtual void ForwardOne(ArrayView<const Base> x,
                            size_t tx1Nnz, const size_t idx[], const Base tx1[],
                            ArrayView<Base> ty1) = 0;

    /***********************************************************************
     *                        Reverse one
     **********************************************************************/

    /**
     * Determines whether or not the first-order reverse mode dense
     * methods can be called.
     *
     * @return true if it is possible to evaluate the first-order reverse mode
     *         using the dense vector format
     */
    virtual bool isReverseOneAvailable() = 0;

    /**
     * Computes results during a reverse mode sweep (adjoints or partial
     * derivatives of independent variables) for the evaluation of the
     * jacobian when the model is used through a user defined 
     * external/atomic AD function.
     * @warning do not used it as a generic reverse mode function!
     */
    template<typename VectorBase>
    inline VectorBase ReverseOne(const VectorBase& tx,
                                 const VectorBase& ty,
                                 const VectorBase& py) {
        const size_t k = 0;
        VectorBase px((k + 1) * Domain());
        this->ReverseOne(tx, ty, px, py);
        return px;
    }

    /**
     * Computes results during a reverse mode sweep (adjoints or partial
     * derivatives of independent variables) for the evaluation of the
     * jacobian when the model is used through a user defined 
     * external/atomic AD function.
     * @warning do not used it as a generic reverse mode function!
     */
    template<typename VectorBase>
    inline void ReverseOne(const VectorBase& tx,
                           const VectorBase& ty,
                           VectorBase& px,
                           const VectorBase& py) {
        this->ReverseOne(ArrayView<const Base>(&tx[0], tx.size()),
                         ArrayView<const Base>(&ty[0], ty.size()),
                         ArrayView<Base>(&px[0], px.size()),
                         ArrayView<const Base>(&py[0], py.size()));
    }

    /**
     * Determines whether or not the first-order reverse mode sparse
     * method can be called.
     *
     * @return true if it is possible to evaluate the first-order reverse mode
     *         using the sparse vector format
     */
    virtual bool isSparseReverseOneAvailable() = 0;

    /**
     * Computes results during a reverse mode sweep (adjoints or partial
     * derivatives of independent variables) for the evaluation of the
     * jacobian when the model is used through a user defined 
     * external/atomic AD function.
     * @warning do not used it as a generic reverse mode function!
     */
    virtual void ReverseOne(ArrayView<const Base> tx,
                            ArrayView<const Base> ty,
                            ArrayView<Base> px,
                            ArrayView<const Base> py) = 0;

    /**
     * Computes results during a reverse mode sweep (adjoints or partial
     * derivatives of independent variables) for the evaluation of the
     * jacobian when the model is used through a user defined 
     * external/atomic AD function.
     * This method version avoids some data copies and can be more efficient.
     * @warning do not used it as a generic reverse mode function!
     * 
     * @param x independent variable vector
     * @param px partial derivatives of the independent variables (same size as x)
     * @param pyNnz the number of non-zeros of the partial derivatives of
     *              the dependent variables (weight functionals)
     * @param idx the locations of the non-zero values the partial 
     *            derivatives of the dependent variables (weight functionals)
     * @param py the non-zero values of the partial derivatives of the 
     *           dependent variables (weight functionals)
     */
    virtual void ReverseOne(ArrayView<const Base> x,
                            ArrayView<Base> px,
                            size_t pyNnz, const size_t idx[], const Base py[]) = 0;

    /***********************************************************************
     *                        Reverse two
     **********************************************************************/

    /**
     * Determines whether or not the second-order reverse mode dense
     * methods can be called.
     *
     * @return true if it is possible to evaluate the second-order reverse mode
     *         using the dense vector format
     */
    virtual bool isReverseTwoAvailable() = 0;

    /**
     * Computes second-order results during a reverse mode sweep (p = 2).
     * This method can be used during the evaluation of the hessian when
     * the model is used through a user defined external/atomic AD function.
     * @warning do not used it as a generic reverse mode function!
     * @warning only the values for px[j * (k+1)] are defined, since
     *          px[j * (k+1) + 1] is not used during the hessian evaluation.
     */
    template<typename VectorBase>
    inline VectorBase ReverseTwo(const VectorBase& tx,
                                 const VectorBase& ty,
                                 const VectorBase& py) {
        const size_t k = 1;
        VectorBase px((k + 1) * Domain());
        this->ReverseTwo(tx, ty, px, py);
        return px;
    }

    /**
     * Computes second-order results during a reverse mode sweep (p = 2).
     * This method can be used during the evaluation of the hessian when
     * the model is used through a user defined external/atomic AD function.
     * @warning do not used it as a generic reverse mode function!
     * @warning only the values for px[j * (k+1)] are defined, since
     *          px[j * (k+1) + 1] is not used during the hessian evaluation.
     */
    template<typename VectorBase>
    inline void ReverseTwo(const VectorBase& tx,
                           const VectorBase& ty,
                           VectorBase& px,
                           const VectorBase& py) {
        this->ReverseTwo(ArrayView<const Base>(&tx[0], tx.size()),
                         ArrayView<const Base>(&ty[0], ty.size()),
                         ArrayView<Base>(&px[0], px.size()),
                         ArrayView<const Base>(&py[0], py.size()));
    }

    /**
     * Determines whether or not the second-order reverse mode sparse
     * methods can be called.
     *
     * @return true if it is possible to evaluate the second-order reverse mode
     *         using the sparse vector format
     */
    virtual bool isSparseReverseTwoAvailable() = 0;

    /**
     * Computes second-order results during a reverse mode sweep (p = 2).
     * This method can be used during the evaluation of the hessian when
     * the model is used through a user defined external/atomic AD function.
     * @warning do not used it as a generic reverse mode function!
     * @warning only the values for px[j * (k+1)] are defined, since
     *          px[j * (k+1) + 1] is not used during the hessian evaluation.
     */
    virtual void ReverseTwo(ArrayView<const Base> tx,
                            ArrayView<const Base> ty,
                            ArrayView<Base> px,
                            ArrayView<const Base> py) = 0;
    /**
     * Computes second-order results during a reverse mode sweep (p = 2).
     * This method can be used during the evaluation of the hessian when
     * the model is used through a user defined external AD function.
     * This method version avoids some data copies and can be more efficient.
     * @warning do not used it as a generic reverse mode function!
     * 
     * @param x independent variable vector
     * @param tx1Nnz the number of non-zeros of the first-order Taylor
     *               coefficients of the independents
     * @param idx the locations of the non-zero values of the first-order
     *            Taylor coefficients of the independents
     * @param tx1 the values of the non-zero first-order Taylor coefficients
     *            of the independents
     * @param px2 second-order partials of the independents
     *            (should have the same size of x)
     * @param py2 second-order partials of the dependents
     *            (should have the size of the dependent variables)
     */
    virtual void ReverseTwo(ArrayView<const Base> x,
                            size_t tx1Nnz, const size_t idx[], const Base tx1[],
                            ArrayView<Base> px2,
                            ArrayView<const Base> py2) = 0;

    /***********************************************************************
     *                        Sparse Jacobians
     **********************************************************************/

    /**
     * Determines whether or not the sparse Jacobian evaluation methods can
     * be called.
     *
     * @return true if it is possible to evaluate the sparse Jacobian
     */
    virtual bool isSparseJacobianAvailable() = 0;

    /**
     * Calculates a Jacobian using sparse methods and saves it into a dense
     * format:
     *  \f[ jac[ i n + j ] = \frac{\partial F_i( x ) }{\partial x_j } \f]  
     * \f$ i = 0 , \ldots , m - 1 \f$ and \f$j = 0 , \ldots , n - 1 \f$.
     * 
     * @param x independent variable vector
     * @return a dense jacobian
     */
    template<typename VectorBase>
    inline VectorBase SparseJacobian(const VectorBase& x) {
        VectorBase jac(Range() * Domain());
        SparseJacobian(ArrayView<const Base>(&x[0], x.size()),
                       ArrayView<Base>(&jac[0], jac.size()));
        return jac;
    }

    /**
     * Calculates a Jacobian using sparse methods and saves it into a dense
     * format:
     *  \f[ jac[ i n + j ] = \frac{\partial F_i( x ) }{\partial x_j } \f]  
     * \f$ i = 0 , \ldots , m - 1 \f$ and \f$j = 0 , \ldots , n - 1 \f$.
     * 
     * @param x independent variable vector
     * @param jac a vector where the dense jacobian will be placed
     */
    template<typename VectorBase>
    inline void SparseJacobian(const VectorBase& x,
                               VectorBase& jac) {
        jac.resize(Range() * Domain());
        SparseJacobian(ArrayView<const Base>(&x[0], x.size()),
                       ArrayView<Base>(&jac[0], jac.size()));
    }

    /**
     * Calculates a Jacobian using sparse methods and saves it into a dense
     * format:
     *  \f[ jac[ i n + j ] = \frac{\partial F_i( x ) }{\partial x_j } \f]  
     * \f$ i = 0 , \ldots , m - 1 \f$ and \f$j = 0 , \ldots , n - 1 \f$.
     * 
     * @param x independent variable array (must have n elements)
     * @param jac an array where the dense jacobian will be placed (must be allocated with at least m * n elements)
     */
    virtual void SparseJacobian(ArrayView<const Base> x,
                                ArrayView<Base> jac) = 0;

    virtual void SparseJacobian(const std::vector<Base> &x,
                                std::vector<Base>& jac,
                                std::vector<size_t>& row,
                                std::vector<size_t>& col) = 0;

    virtual void SparseJacobian(ArrayView<const Base> x,
                                ArrayView<Base> jac,
                                size_t const** row,
                                size_t const** col) = 0;

    /**
     * Determines the sparse Jacobian using a variable number of independent 
     * variable arrays. This method can be useful if the generic model was
     * prepared considering that the independent variables are provided
     * by several arrays.
     * 
     * @param x Contains the several independent variable vectors
     * @param jac The values of the sparse Jacobian in the order provided by
     *            row and col
     * @param row The row indices of the Jacobian values
     * @param col The column indices of the Jacobian values
     */
    virtual void SparseJacobian(const std::vector<const Base*>& x,
                                ArrayView<Base> jac,
                                size_t const** row,
                                size_t const** col) = 0;

    /***********************************************************************
     *                        Sparse Hessians
     **********************************************************************/

    /**
     * Determines whether or not the sparse evaluation of the weighted sum of
     * the Hessians methods can be called.
     *
     * @return true if it is possible to evaluate the sparse weighted sum of
     *         the Hessians
     */
    virtual bool isSparseHessianAvailable() = 0;

    template<typename VectorBase>
    inline VectorBase SparseHessian(const VectorBase& x,
                                    const VectorBase& w) {
        VectorBase hess(Domain() * Domain());
        SparseHessian(ArrayView<const Base>(&x[0], x.size()),
                      ArrayView<const Base>(&w[0], w.size()),
                      ArrayView<Base>(&hess[0], hess.size()));
        return hess;
    }

    template<typename VectorBase>
    inline void SparseHessian(const VectorBase& x,
                              const VectorBase& w,
                              VectorBase& hess) {
        hess.resize(Domain() * Domain());
        SparseHessian(ArrayView<const Base>(&x[0], x.size()),
                      ArrayView<const Base>(&w[0], w.size()),
                      ArrayView<Base>(&hess[0], hess.size()));
    }

    virtual void SparseHessian(ArrayView<const Base> x,
                               ArrayView<const Base> w,
                               ArrayView<Base> hess) = 0;

    virtual void SparseHessian(const std::vector<Base> &x,
                               const std::vector<Base> &w,
                               std::vector<Base>& hess,
                               std::vector<size_t>& row,
                               std::vector<size_t>& col) = 0;

    virtual void SparseHessian(ArrayView<const Base> x,
                               ArrayView<const Base> w,
                               ArrayView<Base> hess,
                               size_t const** row,
                               size_t const** col) = 0;

    /**
     * Determines the sparse Hessian using a variable number of independent 
     * variable arrays. This method can be useful if the generic model was
     * prepared considering that the independent variables are provided
     * by several arrays.
     * 
     * @param x Contains the several independent variable vectors
     * @param w The equation multipliers
     * @param w_size The number of equations
     * @param hess The values of the sparse hessian in the order provided by
     *             row and col
     * @param row The row indices of the hessian values
     * @param col The column indices of the hessian values
     */
    virtual void SparseHessian(const std::vector<const Base*>& x,
                               ArrayView<const Base> w,
                               ArrayView<Base> hess,
                               size_t const** row,
                               size_t const** col) = 0;

    /**
     * Provides a wrapper for this compiled model allowing it to be used as
     * an atomic function. The model must not be deleted while the atomic
     * function is in use.
     * 
     * @return an atomic function wrapper for this model
     */
    virtual CGAtomicGenericModel<Base>& asAtomic() {
        if (_atomic == nullptr) {
            _atomic = new CGAtomicGenericModel<Base>(*this);
        }
        return *_atomic;
    }
};

} // END cg namespace
} // END CppAD namespace

#endif