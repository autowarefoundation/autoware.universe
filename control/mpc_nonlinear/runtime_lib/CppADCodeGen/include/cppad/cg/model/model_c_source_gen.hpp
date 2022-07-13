#ifndef CPPAD_CG_MODEL_C_SOURCE_GEN_INCLUDED
#define CPPAD_CG_MODEL_C_SOURCE_GEN_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *    Copyright (C) 2019 Joao Leal
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
 * Auxiliary internal class
 */
class CompressedVectorInfo {
public:
    std::vector<std::set<size_t> > locations; // location of each index in the compressed array
    std::vector<size_t> indexes; // row or column index
    bool ordered;
};

/**
 * Generates C source code for a model.
 *
 * @author Joao Leal
 */
template<class Base>
class ModelCSourceGen {
    using CGBase = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGBase>;
    using SparsitySetType = std::vector<std::set<size_t> >;
    using TapeVarType = std::pair<size_t, size_t>; // tape independent -> reference orig independent (temporaries only)
public:
    static const std::string FUNCTION_FORWAD_ZERO;
    static const std::string FUNCTION_JACOBIAN;
    static const std::string FUNCTION_HESSIAN;
    static const std::string FUNCTION_FORWARD_ONE;
    static const std::string FUNCTION_REVERSE_ONE;
    static const std::string FUNCTION_REVERSE_TWO;
    static const std::string FUNCTION_SPARSE_JACOBIAN;
    static const std::string FUNCTION_SPARSE_HESSIAN;
    static const std::string FUNCTION_JACOBIAN_SPARSITY;
    static const std::string FUNCTION_HESSIAN_SPARSITY;
    static const std::string FUNCTION_HESSIAN_SPARSITY2;
    static const std::string FUNCTION_SPARSE_FORWARD_ONE;
    static const std::string FUNCTION_SPARSE_REVERSE_ONE;
    static const std::string FUNCTION_SPARSE_REVERSE_TWO;
    static const std::string FUNCTION_FORWARD_ONE_SPARSITY;
    static const std::string FUNCTION_REVERSE_ONE_SPARSITY;
    static const std::string FUNCTION_REVERSE_TWO_SPARSITY;
    static const std::string FUNCTION_INFO;
    static const std::string FUNCTION_ATOMIC_FUNC_NAMES;
protected:
    static const std::string CONST;

    /**
     * Useful class for storing matrix indexes
     */
    class Position {
    public:
        bool defined;
        std::vector<size_t> row;
        std::vector<size_t> col;

        inline Position() :
            defined(false) {
        }

        inline Position(const std::vector<size_t>& r, const std::vector<size_t>& c) :
            defined(true),
            row(r),
            col(c) {
            CPPADCG_ASSERT_KNOWN(r.size() == c.size(), "The number of row indexes must be the same as the number of column indexes.")
        }

        template<class VectorSet>
        inline Position(const VectorSet& elements) :
            defined(true) {
            size_t nnz = 0;
            for (size_t i = 0; i < elements.size(); i++) {
                nnz += elements[i].size();
            }
            row.resize(nnz);
            col.resize(nnz);

            nnz = 0;
            for (size_t i = 0; i < elements.size(); i++) {
                for (size_t it : elements[i]) {
                    row[nnz] = i;
                    col[nnz] = it;
                    nnz++;
                }
            }
        }
    };

    /**
     * Saves sparsity information in more than one format
     */
    class LocalSparsityInfo {
    public:
        /**
         * Calculated sparsity from the model
         * (may differ from the requested sparsity)
         */
        SparsitySetType sparsity;
        // rows (in a custom order)
        std::vector<size_t> rows;
        // columns (in a custom order)
        std::vector<size_t> cols;
    };

    /**
     * Used for coloring
     */
    class Color {
    public:
        /// all row with this color
        std::set<size_t> rows;
        /// maps column indexes to the corresponding row
        std::map<size_t, size_t> column2Row;
        /// maps row indexes to the corresponding columns
        std::map<size_t, std::set<size_t> > row2Columns;
        /// used columns
        std::set<size_t> forbiddenRows;
    };

protected:
    /**
     * the original model
     */
    ADFun<CGBase>& _fun;
    /**
     * Altered model without the loop equations and with extra dependents
     * for the non-indexed temporary variables used by loops
     */
    LoopFreeModel<Base>* _funNoLoops;
    /**
     * loop models
     */
    std::set<LoopModel<Base>*> _loopTapes;
    /**
     * the name of the model
     */
    std::string _name;
    /**
     * the name of the data type used in operations
     */
    const std::string _baseTypeName;
    /**
     * the maximum precision used to print values
     */
    size_t _parameterPrecision;
    /**
     * Typical values of the independent vector
     */
    std::vector<Base> _x;
    /**
     * Whether or not to enable the generation of multithreaded code for the
     * sparse Jacobian and sparse Hessian if possible and requested by the
     * model library (experimental).
     */
    bool _multiThreading;
    /// generate source code for the zero order model evaluation
    bool _zero;
    bool _zeroEvaluated;
    /// generate source code for a dense Jacobian
    bool _jacobian;
    /// generate source code for a dense Hessian
    bool _hessian;
    /// generate source code for a sparse Jacobian
    bool _sparseJacobian;
    /// generate source code for a sparse Hessian
    bool _sparseHessian;
    /**
     * generate source-code for the Hessian sparsity pattern for each
     * equation/dependent
     */
    bool _hessianByEquation;
    /// generate source code for forward first order mode
    bool _forwardOne;
    /// generate source code for reverse first order mode
    bool _reverseOne;
    /// generate source code for reverse second order mode
    bool _reverseTwo;
    /**
     * whether or not the sparse Jacobian should reuse the forward or reverse
     * one functions when _sparseJacobian is true
     */
    bool _sparseJacobianReusesOne;
    /**
     * whether or not the sparse Hessian should reuse the reverse two
     * functions when _sparseHessian is true
     */
    bool _sparseHessianReusesRev2;
    JacobianADMode _jacMode;
    /**
     * Custom Jacobian element indexes
     */
    Position _custom_jac;
    LocalSparsityInfo _jacSparsity;
    /**
     * Custom Hessian element indexes
     */
    Position _custom_hess;
    LocalSparsityInfo _hessSparsity;
    /**
     * Hessian sparsity from the model for each equation
     */
    std::vector<LocalSparsityInfo> _hessSparsities;
    /**
     * The order of the atomic functions
     */
    std::vector<std::string> _atomicFunctions;
    /**
     * Maps each atomic function ID to information regarding how the atomic function is used
     */
    std::map<size_t, AtomicUseInfo<Base> >* _atomicsInfo;
    /**
     * A string cache for code generation
     */
    std::ostringstream _cache;
    /**
     * maximum number of assignments per function (~ lines)
     */
    size_t _maxAssignPerFunc;
    /**
     * the maximum number of operations per variable assignment
     */
    size_t _maxOperationsPerAssignment;
    /**
     *
     */
    std::vector<std::set<size_t> > _relatedDepCandidates;
    /**
     * Maps the column groups of each loop model to the set of columns
     * (loop->group->{columns->{compressed forward 1 position} })
     */
    std::map<LoopModel<Base>*, std::map<size_t, std::map<size_t, std::set<size_t> > > > _loopFor1Groups;
    /**
     * Jacobian columns with a contribution from non loop equations
     *  [var]{compressed forward 1 position}
     */
    std::map<size_t, std::set<size_t> > _nonLoopFor1Elements;
    /**
     * Maps the column groups of each loop model to the set of columns
     * (loop->group->{row->{compressed reverse 1 position} })
     */
    std::map<LoopModel<Base>*, std::map<size_t, std::map<size_t, std::set<size_t> > > > _loopRev1Groups;
    /**
     * Jacobian rows with a contribution from non loop equations
     *  [eq]{compressed reverse 1 position}
     */
    std::map<size_t, std::set<size_t> > _nonLoopRev1Elements;
    /**
     * Maps the row groups of each loop model to the set of rows
     * (loop->group->{rows->{compressed reverse 2 position} })
     */
    std::map<LoopModel<Base>*, std::map<size_t, std::map<size_t, std::set<size_t> > > > _loopRev2Groups;
    /**
     * Hessian rows with a contribution from non loop equations
     *  [var]{compressed reverse 2 position}
     */
    std::map<size_t, std::set<size_t> > _nonLoopRev2Elements;
    /**
     *
     */
    JobTimer* _jobTimer;
    /**
     * Generated source code (maps file names to content)
     */
    std::map<std::string, std::string> _sources;
public:

    /**
     * Creates a new C language compilation helper for a model
     *
     * @param fun The ADFun with the taped model (should only be deleted
     *            after this object)
     * @param model The model name (must be a valid C function name)
     */
    ModelCSourceGen(ADFun<CppAD::cg::CG<Base> >& fun,
                    std::string model) :
        _fun(fun),
        _funNoLoops(nullptr),
        _name(std::move(model)),
        _baseTypeName(ModelCSourceGen<Base>::baseTypeName()),
        _parameterPrecision(std::numeric_limits<Base>::digits10),
        _multiThreading(true),
        _zero(true),
        _zeroEvaluated(false),
        _jacobian(false),
        _hessian(false),
        _sparseJacobian(false),
        _sparseHessian(false),
        _hessianByEquation(false),
        _forwardOne(false),
        _reverseOne(false),
        _reverseTwo(false),
        _sparseJacobianReusesOne(true),
        _sparseHessianReusesRev2(true),
        _jacMode(JacobianADMode::Automatic),
        _atomicsInfo(nullptr),
        _maxAssignPerFunc(20000),
        _maxOperationsPerAssignment(1000),
        _jobTimer(nullptr) {

        CPPADCG_ASSERT_KNOWN(!_name.empty(), "Model name cannot be empty");
        CPPADCG_ASSERT_KNOWN((_name[0] >= 'a' && _name[0] <= 'z') ||
                             (_name[0] >= 'A' && _name[0] <= 'Z'),
                             "Invalid model name character");
        for (size_t i = 1; i < _name.size(); i++) {
            char c = _name[i];
            CPPADCG_ASSERT_KNOWN((c >= 'a' && c <= 'z') ||
                                 (c >= 'A' && c <= 'Z') ||
                                 (c >= '0' && c <= '9') ||
                                 c == '_'
                                 , "Invalid model name character");
        }
    }

    ModelCSourceGen(const ModelCSourceGen&) = delete;
    ModelCSourceGen& operator=(const ModelCSourceGen&) = delete;

    /**
     * Provides the model name which should be a valid C function name.
     *
     * @return the model name
     */
    inline const std::string& getName() const {
        return _name;
    }

    /**
     * Defines typical values for the independent variable vector. These
     * values can be useful when there is a need to call atomic functions,
     * since they may allow to reduce some operations.
     *
     * @param x The typical values. An empty vector removes the currently
     *          defined values.
     */
    template<class VectorBase>
    inline void setTypicalIndependentValues(const VectorBase& x) {
        CPPAD_ASSERT_KNOWN(x.size() == 0 || x.size() == _fun.Domain(),
                           "Invalid independent variable vector size")
        _x.resize(x.size());
        for (size_t i = 0; i < x.size(); i++) {
            _x[i] = x[i];
        }
    }

    inline void setRelatedDependents(const std::vector<std::set<size_t> >& relatedDepCandidates) {
        _relatedDepCandidates = relatedDepCandidates;
    }

    inline const std::vector<std::set<size_t> >& getRelatedDependents() const {
        return _relatedDepCandidates;
    }

    /**
     * Provides the maximum precision used to print constant values in the
     * generated source code
     *
     * @return the maximum number of digits
     */
    virtual size_t getParameterPrecision() const {
        return _parameterPrecision;
    }

    /**
     * Defines the maximum precision used to print constant values in the
     * generated source code.
     *
     * @param p the maximum number of digits
     */
    virtual void setParameterPrecision(size_t p) {
        _parameterPrecision = p;
    }

    /**
     * Returns whether or not multithreading directives can be generated to
     * parallelize the sparse Jacobian and sparse Hessian evaluation.
     * Multithreaded code is only generated if requested by the model library.
     * For the sparse Jacobian, the _sparseJacobianReusesOne must be enabled,
     * at least one of _forwardOne and _reverseOne must be enabled, and loop
     * detection must be disabled.
     * For the sparse Hessian, the _sparseHessianReusesRev2 and _reverseTwo
     * must be enabled and loop detection must be disabled.
     *
     * @return whether or not multithreading can be used for this model
     */
    inline bool isMultiThreading() const {
        return _multiThreading;
    }

    /**
     * Defines whether or not multithreading directives can be generated to
     * parallelize the sparse Jacobian and sparse Hessian evaluation.
     * Multithreaded code is only generated if requested by the model library.
     * For the sparse Jacobian, the _sparseJacobianReusesOne must be enabled,
     * at least one of _forwardOne and _reverseOne must be enabled, and loop
     * detection must be disabled.
     * For the sparse Hessian, the _sparseHessianReusesRev2 and _reverseTwo
     * must be enabled and loop detection must be disabled.
     *
     * @param multiThreading whether or not multithreading can be used for this
     *                       model
     */
    inline void setMultiThreading(bool multiThreading) {
        _multiThreading = multiThreading;
    }

    inline bool isJacobianMultiThreadingEnabled() const {
        return _multiThreading && _loopTapes.empty() && _sparseJacobian && _sparseJacobianReusesOne && (_forwardOne || _reverseOne);
    }

    inline bool isHessianMultiThreadingEnabled() const {
        return _multiThreading && _loopTapes.empty() && _sparseHessian && _sparseHessianReusesRev2 && _reverseTwo;
    }

    /**
     * Determines whether or not to generate source-code for a function
     * that evaluates a dense Hessian.
     *
     * @return true if source-code for a dense Hessian should be created,
     *         false otherwise
     */
    inline bool isCreateHessian() const {
        return _hessian;
    }

    /**
     * Defines whether or not to generate source-code for a function
     * that evaluates a dense Hessian.
     *
     * @param create true if source-code for a dense Hessian should be
     *               created, false otherwise
     */
    inline void setCreateHessian(bool create) {
        _hessian = create;
    }

    /**
     * Provides the Automatic Differentiation mode used to generate the
     * source code for the Jacobian
     *
     * @return the Automatic Differentiation mode
     */
    inline JacobianADMode getJacobianADMode() const {
        return _jacMode;
    }

    /**
     * Defines the Automatic Differentiation mode used to generate the
     * source code for the Jacobian
     *
     * @param mode the Automatic Differentiation mode
     */
    inline void setJacobianADMode(JacobianADMode mode) {
        _jacMode = mode;
    }

    /**
     * Determines whether or not to generate source-code for a function
     * that evaluates a dense Jacobian.
     *
     * @return true if source-code for a dense Jacobian should be created,
     *         false otherwise
     */
    inline bool isCreateJacobian() const {
        return _jacobian;
    }

    /**
     * Defines whether or not to generate source-code for a function
     * that evaluates a dense Jacobian.
     *
     * @param create true if source-code for a dense Jacobian should be
     *               created, false otherwise
     */
    inline void setCreateJacobian(bool create) {
        _jacobian = create;
    }

    /**
     * Determines whether or not to generate source-code for a function
     * that evaluates a sparse Hessian.
     * If ReverseTwo and SparseHessianReusesRev2 are also enabled the generated
     * source-code will use the individual generated functions from the
     * second-order reverse mode.
     * Enabling the generation of individuals functions for reverse-mode
     * can have a negative impact on the performance of the evaluation of the
     * sparse hessian since Hessian symmetry will not be exploited. To
     * improve performance one can request only the upper or lower elements
     * of the hessian using setCustomSparseHessianElements().
     *
     * @see setCustomSparseHessianElements()
     * @see setSparseHessianReusesRev2()
     *
     * @return true if source-code for a sparse Hessian should be created,
     *         false otherwise
     */
    inline bool isCreateSparseHessian() const {
        return _sparseHessian;
    }

    /**
     * Defines whether or not to generate source-code for a function
     * that evaluates a sparse Hessian.
     * If ReverseTwo and SparseHessianReusesRev2 are also enabled the generated
     * source-code will use the individual generated functions from the
     * second-order reverse mode.
     * Enabling the generation of individuals functions for reverse-mode
     * can have a negative impact on the performance of the evaluation of the
     * sparse hessian since Hessian symmetry will not be exploited. To
     * improve performance one can request only the upper or lower elements
     * of the hessian using setCustomSparseHessianElements().
     *
     * @see setCustomSparseHessianElements()
     * @see setSparseHessianReusesRev2()
     *
     * @param create true if source-code for a sparse Hessian should be
     *               created, false otherwise
     */
    inline void setCreateSparseHessian(bool create) {
        _sparseHessian = create;
    }

    /**
     * Determines whether or not the sparse Hessian should reuse functions
     * generated for the reverse two pass.
     * The sparse Hessian will only make use of the reverse two mode if both
     * this flag is set to true and the reverseTwo mode is enabled.
     *
     * @return true if the functions created for reverse 2 mode should be
     *         reused to determine the sparse Hessian, false otherwise
     */
    inline bool isSparseHessianReusesRev2() const {
        return _sparseHessianReusesRev2;
    }

    /**
     * Defines whether or not the sparse Hessian should reuse functions
     * generated for the reverse two pass.
     * The sparse Hessian will only make use of the reverse two mode if both
     * this flag is set to true and the reverseTwo mode is enabled.
     *
     * @param reuse true if the functions created for reverse 2 mode should be
     *              reused to determine the sparse Hessian, false otherwise
     */
    inline void setSparseHessianReusesRev2(bool reuse) {
        _sparseHessianReusesRev2 = reuse;
    }

    /**
     * Determines whether or not to generate source-code for a function that
     * provides the Hessian sparsity pattern for each equation/dependent,
     * when the sparse hessian creation is enabled.
     * Even if this flag is set to false the function can still be generated
     * if the second-order reverse mode is enabled.
     *
     * @return true if source-code for a Hessians sparsities patterns should
     *         be created, false otherwise
     */
    inline bool isCreateHessianSparsityByEquation() const {
        return _hessianByEquation;
    }

    /**
     * Defines whether or not to generate source-code for a function that
     * provides the Hessian sparsity pattern for each equation/dependent,
     * when the sparse hessian creation is enabled.
     * Even if this flag is set to false the function can still be generated
     * if the second-order reverse mode is enabled.
     *
     * @param create true if source-code for a Hessians sparsities should be
     *               created, false otherwise
     */
    inline void setCreateHessianSparsityByEquation(bool create) {
        _hessianByEquation = create;
    }

    /**
     * Determines whether or not to generate source-code for a function
     * that evaluates a sparse Jacobian. If ReverseOne or ForwardOne
     * functions are enabled, then the sparse Jacobian evaluation might
     * use those functions.
     * Enabling the generation of individuals functions for reverse-mode
     * can have a small negative impact on the performance of the evaluation of
     * the parse Jacobian.

     * @see setCustomSparseJacobianElements()
     *
     * @return true if source-code for a sparse Jacobian should be created,
     *         false otherwise
     */
    inline bool isCreateSparseJacobian() const {
        return _sparseJacobian;
    }

    /**
     * Defines whether or not to generate source-code for a function
     * that evaluates a sparse Jacobian. If ReverseOne or ForwardOne
     * functions are enabled, then the sparse Jacobian evaluation might
     * use those functions.
     * Enabling the generation of individuals functions for reverse-mode
     * can have a small negative impact on the performance of the evaluation of
     * the parse Jacobian.

     * @see setCustomSparseJacobianElements()
     *
     * @param create true if source-code for a sparse Jacobian should be
     *               created, false otherwise
     */
    inline void setCreateSparseJacobian(bool create) {
        _sparseJacobian = create;
    }

    /**
     * Determines whether or not the sparse Jacobian should reuse functions
     * generated for the forward one or reverse one pass.
     * The sparse Jacobian will only make use of these functions if both
     * this flag is set to true and one of the forward one and reverse one
     * mode is enabled.
     *
     * @return true if the functions created for forward or reverse 1 mode
     *         should be reused to determine the sparse Jacobian, false otherwise
     */
    inline bool isSparseJacobianReuse1stOrderPasses() const {
        return _sparseJacobianReusesOne;
    }

    /**
     * Defines whether or not the sparse Jacobian should reuse functions
     * generated for the forward one or reverse one pass.
     * The sparse Jacobian will only make use of these functions if both
     * this flag is set to true and one of the forward one and reverse one
     * mode is enabled.
     *
     * @param reuse true if the functions created for forward or reverse 1 mode
     *              should be reused to determine the sparse Jacobian,
     *              false otherwise
     */
    inline void setSparseJacobianReuse1stOrderPasses(bool reuse) {
        _sparseJacobianReusesOne = reuse;
    }

    /**
     * Determines whether or not to generate source-code for a function
     * that evaluates the original model.
     *
     * @return true if source-code for the original model should be created,
     *         false otherwise
     */
    inline bool isCreateForwardZero() const {
        return _zero;
    }

    /**
     * Defines whether or not to generate source-code for a function
     * that evaluates the original model.
     *
     * @param create true if source-code for the original model should be
     *                created, false otherwise
     */
    inline void setCreateForwardZero(bool create) {
        _zero = create;
    }

    /**
     * Determines whether or not to generate source-code for the
     * first-order forward mode that is used for the evaluation of the
     * Jacobian when the model is used through a user defined atomic
     * AD function.
     * Enabling the generation of individuals functions for forward-mode
     * might have a small negative impact on the performance of the evaluation
     * of the sparse Jacobian (if forward mode is selected).
     *
     * @see isCreateSparseJacobian()
     *
     * @return true if the generation of the source for first-order forward
     *         mode is enabled, false otherwise.
     */
    inline bool isCreateSparseForwardOne() const {
        return _forwardOne;
    }

    /**
     * Defines whether or not to generate source-code for the
     * first-order forward mode that is used for the evaluation of the
     * Jacobian when the model is used through a user defined atomic
     * AD function.
     * Enabling the generation of individuals functions for forward-mode
     * might have a small negative impact on the performance of the evaluation
     * of the sparse Jacobian (if forward-mode is selected).
     *
     * @see setCreateSparseJacobian()
     *
     * @param create true if the generation of the source for first-order
     *               forward mode is enabled, false otherwise.
     */
    inline void setCreateForwardOne(bool create) {
        _forwardOne = create;
    }

    /**
     * Determines whether or not to generate source-code for the
     * first-order reverse mode that is used for the evaluation of the
     * Jacobian when the model is used through a user defined atomic
     * AD function.
     * Enabling the generation of individuals functions for reverse-mode
     * might have a small negative impact on the performance of the evaluation
     * of the sparse Jacobian (if reverse-mode is selected).
     *
     * @see isCreateSparseJacobian()
     *
     * @return true if the generation of the source for first-order reverse
     *         mode is enabled, false otherwise.
     */
    inline bool isCreateReverseOne() const {
        return _reverseOne;
    }

    /**
     * Determines whether or not to generate source-code for the
     * first-order reverse mode that is used for the evaluation of the
     * Jacobian when the model is used through a user defined atomic
     * AD function.
     * Enabling the generation of individuals functions for reverse-mode
     * might have a small negative impact on the performance of the evaluation
     * of the sparse Jacobian (if reverse-mode is selected).
     *
     * @see setCreateSparseJacobian()
     *
     * @param create true if the source for first-order reverse mode functions
     *               should be generated, false otherwise.
     */
    inline void setCreateReverseOne(bool create) {
        _reverseOne = create;
    }

    /**
     * Determines whether or not to generate source-code for the
     * second-order reverse mode that is used for the evaluation of the
     * hessian when the model is used through a user defined atomic
     * AD function.
     * Enabling the generation of individuals functions for reverse-mode
     * can have a negative impact on the performance of the evaluation of the
     * sparse hessian.
     *
     * Warning: only the values for px[j * (k+1)] will be defined, since
     *          px[j * (k+1) + 1] is not used during the hessian evaluation.
     *
     * @return true if the generation of the source for second-order reverse
     *         mode is enabled, false otherwise.
     */
    inline bool isCreateReverseTwo() const {
        return _reverseOne;
    }

    /**
     * Defines whether or not to enable the generation of the source-code
     * for the second-order reverse mode that is used for the evaluation
     * of the hessian when the model is used through a user defined atomic
     * AD function.
     * Enabling the generation of individuals functions for reverse-mode
     * can have a negative impact on the performance of the evaluation of the
     * hessian. To improve performance one can request only the upper or
     * lower elements of the hessian using setCustomSparseHessianElements()
     * and later only request those elements through the outer model (ADFun).
     *
     * Warning: only the values for px[j * (k+1)] will be defined, since
     *          px[j * (k+1) + 1] is not used during the hessian evaluation.
     *
     * @param create true to enable the generation of the source for
     *               second-order reverse mode, false otherwise.
     */
    inline void setCreateReverseTwo(bool create) {
        _reverseTwo = create;
    }

    /**
     * Specifies a user defined Jacobian sparsity to be computed.
     * The elements can be provided in any order as long as they are a subset
     * of the full Jacobian sparsity pattern.
     *
     * @param row The indices of the equations/dependents for the sparsity
     *            pattern in the order they will be provided
     * @param col The indices of the independent variables for the sparsity
     *            pattern in the order they will be provided
     */
    inline void setCustomSparseJacobianElements(const std::vector<size_t>& row,
                                                const std::vector<size_t>& col) {
        _custom_jac = Position(row, col);
    }

    /**
     * Specifies a user defined Jacobian sparsity to be computed.
     * The elements must be a subset of the full Jacobian sparsity pattern.
     *
     * @tparam VectorSet A vector of sets
     * @param elements The indices of the independent variables (defined in a
     *                 set) for each equation/dependent variable (the vector
     *                 must not be larger than the number dependent variables).
     */
    template<class VectorSet>
    inline void setCustomSparseJacobianElements(const VectorSet& elements) {
        _custom_jac = Position(elements);
    }

    /**
     * Specifies a user defined Hessian sparsity to be computed.
     * This can be used, for instance, to request only half of the Hessian
     * to be computed due to its symmetry.
     * The elements can be provided in any order as long as they are a subset
     * of the full Hessian sparsity pattern.
     *
     * @param row The row indices of the independent variables for the sparsity
     *            pattern in the order they will be provided
     * @param col The column indices of the independent variables for the
     *            sparsity pattern in the order they will be provided
     */
    inline void setCustomSparseHessianElements(const std::vector<size_t>& row,
                                               const std::vector<size_t>& col) {
        _custom_hess = Position(row, col);
    }

    /**
     * Specifies a user defined Hessian sparsity to be computed.
     * This can be used, for instance, to request only half of the Hessian
     * to be computed due to its symmetry.
     * The elements must be a subset of the full Hessian sparsity pattern.
     *
     * @tparam VectorSet A vector of sets
     * @param elements The indices of the independent variables for the sparsity
     *                 pattern (the vector must not be larger than the number
     *                 independent variables).
     */
    template<class VectorSet>
    inline void setCustomSparseHessianElements(const VectorSet& elements) {
        _custom_hess = Position(elements);
    }

    /**
     * The maximum number of assignment per generated function.
     * Zero means it is disabled (no limit).
     * Note that it is not possible to split some function (e.g., containing loops) and, therefore, this
     * limit can be violated.
     *
     * @return The maximum number of assignments per file/function
     */
    inline size_t getMaxAssignmentsPerFunc() const {
        return _maxAssignPerFunc;
    }

    /**
     * Sets the maximum number of assignment per generated function.
     * Zero means it is disabled (no limit).
     * By setting a limit, it is possible to reduce the compiler workload by having multiple file/function
     * instead of a very large one.
     * Note that it is not possible to split some function (e.g., containing loops) and, therefore, this
     * limit can be violated.
     *
     * @param maxAssignPerFunc The maximum number of assignments per file/function
     */
    inline void setMaxAssignmentsPerFunc(size_t maxAssignPerFunc) {
        _maxAssignPerFunc = maxAssignPerFunc;
    }

    /**
     * The maximum number of operations per variable assignment.
     *
     * @return The maximum number of operations per variable assignment
     */
    inline size_t getMaxOperationsPerAssignment() const {
        return _maxOperationsPerAssignment;
    }

    /**
     * Defines the maximum number of operations per variable assignment.
     * Defining a limit can reduce the memory required for compilation of the source code.
     *
     * @param maxOperationsPerAssignment  The maximum number of operations per variable assignment.
     */
    inline void setMaxOperationsPerAssignment(size_t maxOperationsPerAssignment) {
        _maxOperationsPerAssignment = maxOperationsPerAssignment;
    }

    inline virtual ~ModelCSourceGen() {
        delete _funNoLoops;
        delete _atomicsInfo;

        for (LoopModel<Base>* it : _loopTapes) {
            delete it;
        }
    }

public:
    static inline std::string baseTypeName();

    template<class T>
    static void generateFunctionDeclarationSource(std::ostringstream& cache,
                                                  const std::string& model_function,
                                                  const std::string& suffix,
                                                  const std::map<size_t, T>& elements,
                                                  const std::string& argsDcl);

protected:

    virtual VariableNameGenerator<Base>* createVariableNameGenerator(const std::string& depName = "y",
                                                                     const std::string& indepName = "x",
                                                                     const std::string& tmpName = "v",
                                                                     const std::string& tmpArrayName = "array");

    const std::map<std::string, std::string>& getSources(MultiThreadingType multiThreadingType,
                                                         JobTimer* timer);

    virtual void generateSources(MultiThreadingType multiThreadingType,
                                 JobTimer* timer = nullptr);

    virtual void generateLoops();

    virtual void generateInfoSource();

    virtual void generateAtomicFuncNames();

    virtual bool isAtomicsUsed();

    virtual const std::map<size_t, AtomicUseInfo<Base> >& getAtomicsInfo();

    /***********************************************************************
     * zero order (the original model)
     **********************************************************************/

    virtual void generateZeroSource();

    /**
     * Generates the operation graph for the zero order model with loops
     */
    virtual std::vector<CGBase> prepareForward0WithLoops(CodeHandler<Base>& handler,
                                                         const std::vector<CGBase>& x);

    /***********************************************************************
     * Jacobian
     **********************************************************************/

    virtual void generateJacobianSource();

    virtual void generateSparseJacobianSource(MultiThreadingType multiThreadingType);

    virtual void generateSparseJacobianSource(bool forward);

    virtual void generateSparseJacobianForRevSource(bool forward,
                                                    MultiThreadingType multiThreadingType);

    virtual std::string generateSparseJacobianForRevSingleThreadSource(const std::string& functionName,
                                                                       std::map<size_t, CompressedVectorInfo> jacInfo,
                                                                       size_t maxCompressedSize,
                                                                       const std::string& functionRevFor,
                                                                       const std::string& revForSuffix,
                                                                       bool forward);

    virtual std::string generateSparseJacobianForRevMultiThreadSource(const std::string& functionName,
                                                                      std::map<size_t, CompressedVectorInfo> jacInfo,
                                                                      size_t maxCompressedSize,
                                                                      const std::string& functionRevFor,
                                                                      const std::string& revForSuffix,
                                                                      bool forward,
                                                                      MultiThreadingType multiThreadingType);
    /**
     * Generates a sparse Jacobian using loops.
     *
     * The original model is split into two models:
     *   - one for the repeated equations
     * \f[ y_i = f(x_{l(j)}, x_v, z_k) \f]
     *   - and another for the equations which do not belong in a loop and the
     *   non-indexed temporary variables (\f$z\f$) used by \f$f\f$.
     * \f[ z_k = g_k(x_v) \f]
     *
     * The jacobian elements for the equations in loops are evaluated as:
     * \f[ \frac{\mathrm{d} y_i}{\mathrm{d} x_v} =
     *        \sum_k \left( \frac{\partial f_i}{\partial z_k} \frac{\partial z_k}{\partial x_v} \right) +
     *        \sum_j \left( \frac{\partial f_i}{\partial x_{l(j)}} \frac{\partial x_{l(j)}}{\partial x_v} \right) +
     *        \frac{\partial f_i}{\partial x_v}
     * \f]
     *
     * @param handler The operation graph handler
     * @param indVars The independent variables
     * @return the operation graph for the compressed jacobin with loops
     */
    virtual std::vector<CGBase> prepareSparseJacobianWithLoops(CodeHandler<Base>& handler,
                                                               const std::vector<CGBase>& x,
                                                               bool forward);

    inline void prepareSparseJacobianRowWithLoops(CodeHandler<Base>& handler,
                                                  LoopModel<Base>& lModel,
                                                  size_t tapeI,
                                                  const loops::JacobianWithLoopsRowInfo& rowInfo,
                                                  const std::vector<std::map<size_t, CGBase> >& dyiDxtape,
                                                  const std::vector<std::map<size_t, CGBase> >& dzDx,
                                                  const CGBase& py,
                                                  IndexOperationNode<Base>& iterationIndexOp,
                                                  std::vector<loops::IfElseInfo<Base> >& ifElses,
                                                  size_t& jacLE,
                                                  std::vector<std::pair<CG<Base>, IndexPattern*> >& indexedLoopResults,
                                                  std::set<size_t>& allLocations);

    inline void analyseSparseJacobianWithLoops(const std::vector<size_t>& rows,
                                               const std::vector<size_t>& cols,
                                               const std::vector<size_t>& location,
                                               SparsitySetType& noLoopEvalSparsity,
                                               std::vector<std::map<size_t, std::set<size_t> > >& noLoopEvalLocations,
                                               std::map<LoopModel<Base>*, SparsitySetType>& loopsEvalSparsities,
                                               std::map<LoopModel<Base>*, std::vector<loops::JacobianWithLoopsRowInfo> >& loopEqInfo);


    virtual void generateSparseJacobianWithLoopsSourceFromForRev(const std::map<size_t, CompressedVectorInfo>& jacInfo,
                                                                 size_t maxCompressedSize,
                                                                 const std::string& localFunctionTypeName,
                                                                 const std::string& suffix,
                                                                 const std::string& keyName,
                                                                 const std::map<size_t, std::set<size_t> >& nonLoopElements,
                                                                 const std::map<LoopModel<Base>*, std::map<size_t, std::map<size_t, std::set<size_t> > > >& loopGroups,
                                                                 void (*generateLocalFunctionName)(std::ostringstream& cache, const std::string& modelName, const LoopModel<Base>& loop, size_t g));

    inline virtual void generateFunctionNameLoopFor1(std::ostringstream& cache,
                                                     const LoopModel<Base>& loop,
                                                     size_t g);

    inline static void generateFunctionNameLoopFor1(std::ostringstream& cache,
                                                    const std::string& modelName,
                                                    const LoopModel<Base>& loop,
                                                    size_t g);

    inline virtual void generateFunctionNameLoopRev1(std::ostringstream& cache,
                                                     const LoopModel<Base>& loop,
                                                     size_t i);

    inline static void generateFunctionNameLoopRev1(std::ostringstream& cache,
                                                    const std::string& modelName,
                                                    const LoopModel<Base>& loop,
                                                    size_t i);

    /***********************************************************************
     * Hessian
     **********************************************************************/

    virtual void generateHessianSource();

    virtual void generateSparseHessianSource(MultiThreadingType multiThreadingType);

    virtual void generateSparseHessianSourceDirectly();

    virtual void generateSparseHessianSourceFromRev2(MultiThreadingType multiThreadingType);

    virtual std::string generateSparseHessianRev2SingleThreadSource(const std::string& functionName,
                                                                    std::map<size_t, CompressedVectorInfo> hessInfo,
                                                                    size_t maxCompressedSize,
                                                                    const std::string& functionRev2,
                                                                    const std::string& rev2Suffix);

    virtual std::string generateSparseHessianRev2MultiThreadSource(const std::string& functionName,
                                                                   std::map<size_t, CompressedVectorInfo> hessInfo,
                                                                   size_t maxCompressedSize,
                                                                   const std::string& functionRev2,
                                                                   const std::string& rev2Suffix,
                                                                   MultiThreadingType multiThreadingType);

    virtual void determineSecondOrderElements4Eval(std::vector<size_t>& userRows,
                                                   std::vector<size_t>& userCols);

    /**
     * Loops
     */
    /**
     * Generates a sparse Hessian using loops.
     *
     * The original model is split into two models:
     *   - one for the repeated equations
     * \f[ y_i = f(x_{l(j)}, x_v, z_k) \f]
     *   - and another for the equations which do not belong in a loop and
     *     the non-indexed temporary variables (\f$z\f$) used by \f$f\f$.
     * \f[ z_k = g_k(x_v) \f]
     *
     * The Hessian elements for the equations in loops are evaluated as:
     * \f[ \frac{\mathrm{d}^2 y_i}{\partial x_w \partial x_v} =
     *        \sum_k \left( \frac{\partial^2 f_i}{\partial x_w \partial z_k} \frac{\partial z_k}{\partial x_v} +
     *                      \frac{\partial f_i}{\partial z_k} \frac{\partial^2 z_k}{\partial x_w \partial x_v}
     *               \right) +
     *        \sum_j \left( \frac{\partial^2 f_i}{\partial x_w \partial x_{l(j)}} \frac{\partial x_{l(j)}}{\partial x_v} \right) +
     *        \frac{\partial^2 f_i}{\partial x_w \partial x_v}
     * \f]
     *
     * @param handler The operation graph handler
     * @param indVars The independent variables
     * @return the operation graph for the compressed jacobin with loops
     */
    virtual std::vector<CGBase> prepareSparseHessianWithLoops(CodeHandler<Base>& handler,
                                                              std::vector<CGBase>& indVars,
                                                              std::vector<CGBase>& w,
                                                              const std::vector<size_t>& lowerHessRows,
                                                              const std::vector<size_t>& lowerHessCols,
                                                              const std::vector<size_t>& lowerHessOrder,
                                                              const std::map<size_t, size_t>& duplicates);

    inline void analyseSparseHessianWithLoops(const std::vector<size_t>& lowerHessRows,
                                              const std::vector<size_t>& lowerHessCols,
                                              const std::vector<size_t>& lowerHessOrder,
                                              SparsitySetType& noLoopEvalJacSparsity,
                                              SparsitySetType& noLoopEvalHessSparsity,
                                              std::vector<std::map<size_t, std::set<size_t> > >& noLoopEvalHessLocations,
                                              std::map<LoopModel<Base>*, loops::HessianWithLoopsInfo<Base> >& loopHessInfo,
                                              bool useSymmetry);

    inline virtual void generateSparseHessianWithLoopsSourceFromRev2(const std::map<size_t, CompressedVectorInfo>& hessInfo,
                                                                     size_t maxCompressedSize);

    inline virtual void generateFunctionNameLoopRev2(std::ostringstream& cache,
                                                     const LoopModel<Base>& loop,
                                                     size_t g);

    static inline void generateFunctionNameLoopRev2(std::ostringstream& cache,
                                                    const std::string& modelName,
                                                    const LoopModel<Base>& loop,
                                                    size_t g);

    /***********************************************************************
     * Sparsities for forward/reverse
     **********************************************************************/

    virtual void generateSparsity1DSource(const std::string& function,
                                          const std::vector<size_t>& sparsity);

    virtual void generateSparsity2DSource(const std::string& function,
                                          const LocalSparsityInfo& sparsity);

    virtual void generateSparsity2DSource2(const std::string& function,
                                           const std::vector<LocalSparsityInfo>& sparsities);

    virtual void generateSparsity1DSource2(const std::string& function,
                                           const std::map<size_t, std::vector<size_t> >& rows);

    /***********************************************************************
     * Forward 1 mode
     **********************************************************************/

    virtual void generateSparseForwardOneSources();

    virtual void generateSparseForwardOneSourcesWithAtomics(const std::map<size_t, std::vector<size_t> >& elements);

    virtual void generateSparseForwardOneSourcesNoAtomics(const std::map<size_t, std::vector<size_t> >& elements);

    virtual void generateForwardOneSources();

    virtual void prepareSparseForwardOneWithLoops(const std::map<size_t, std::vector<size_t> >& elements);

    virtual void createForwardOneWithLoopsNL(CodeHandler<Base>& handler,
                                             size_t j,
                                             std::vector<CG<Base> >& jacCol);


    inline static std::map<size_t, std::map<size_t, CG<Base> > > generateLoopFor1Jac(ADFun<CGBase>& fun,
                                                                                     const SparsitySetType& sparsity,
                                                                                     const SparsitySetType& evalSparsity,
                                                                                     const std::vector<CGBase>& xl,
                                                                                     bool constainsAtomics);

    /***********************************************************************
     * Reverse 1 mode
     **********************************************************************/

    virtual void generateSparseReverseOneSources();

    virtual void generateSparseReverseOneSourcesWithAtomics(const std::map<size_t, std::vector<size_t> >& elements);

    virtual void generateSparseReverseOneSourcesNoAtomics(const std::map<size_t, std::vector<size_t> >& elements);

    virtual void generateReverseOneSources();

    virtual void prepareSparseReverseOneWithLoops(const std::map<size_t, std::vector<size_t> >& elements);

    virtual void createReverseOneWithLoopsNL(CodeHandler<Base>& handler,
                                             size_t i,
                                             std::vector<CG<Base> >& jacRow);

    inline static std::vector<std::map<size_t, CGBase> > generateLoopRev1Jac(ADFun<CGBase>& fun,
                                                                             const SparsitySetType& sparsity,
                                                                             const SparsitySetType& evalSparsity,
                                                                             const std::vector<CGBase>& xl,
                                                                             bool constainsAtomics);

    /***********************************************************************
     * Reverse 2 mode
     **********************************************************************/

    virtual void generateSparseReverseTwoSources();

    virtual void generateSparseReverseTwoSourcesWithAtomics(const std::map<size_t, std::vector<size_t> >& elements);

    virtual void generateSparseReverseTwoSourcesNoAtomics(const std::map<size_t, std::vector<size_t> >& elements,
                                                          const std::vector<size_t>& evalRows,
                                                          const std::vector<size_t>& evalCols);

    virtual void generateReverseTwoSources();

    virtual void generateGlobalDirectionalFunctionSource(const std::string& function,
                                                         const std::string& function2_suffix,
                                                         const std::string& function_sparsity,
                                                         const std::map<size_t, std::vector<size_t> >& elements);

    /**
     * Loops
     */
    virtual void prepareSparseReverseTwoWithLoops(const std::map<size_t, std::vector<size_t> >& elements);

    /***********************************************************************
     * Sparsities
     **********************************************************************/

    virtual void determineJacobianSparsity();

    virtual void generateJacobianSparsitySource();

    virtual void determineHessianSparsity();

    /**
     * Determines groups of rows from a sparsity pattern which do not share
     * the same columns
     *
     * @param columns the column indexes of interest (all others are ignored);
     *                an empty set means all columns
     * @param sparsity The sparsity pattern to color
     * @return the colors
     */
    inline std::vector<ModelCSourceGen<Base>::Color> colorByRow(const std::set<size_t>& columns,
                                                                const SparsitySetType& sparsity);

    virtual void generateHessianSparsitySource();


    static inline std::map<size_t, std::vector<std::set<size_t> > > determineOrderByCol(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                        const LocalSparsityInfo& sparsity);

    static inline std::map<size_t, std::vector<std::set<size_t> > > determineOrderByCol(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                        const std::vector<size_t>& userRows,
                                                                                        const std::vector<size_t>& userCols);

    static inline std::vector<std::set<size_t> > determineOrderByCol(size_t col,
                                                                     const std::vector<size_t>& colElements,
                                                                     const std::vector<size_t>& userRows,
                                                                     const std::vector<size_t>& userCols);

    static inline std::map<size_t, std::vector<std::set<size_t> > > determineOrderByRow(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                        const LocalSparsityInfo& sparsity);

    static inline std::map<size_t, std::vector<std::set<size_t> > > determineOrderByRow(const std::map<size_t, std::vector<size_t> >& elements,
                                                                                        const std::vector<size_t>& userRows,
                                                                                        const std::vector<size_t>& userCols);

    static inline std::vector<std::set<size_t> > determineOrderByRow(size_t row,
                                                                     const std::vector<size_t>& rowsElements,
                                                                     const std::vector<size_t>& userRows,
                                                                     const std::vector<size_t>& userCols);

    /***********************************************************************
     * Multi-threading
     **********************************************************************/

    /**
     *
     */
    static void printFileStartPThreads(std::ostringstream& cache,
                                       const std::string& baseTypeName);

    static void printFunctionStartPThreads(std::ostringstream& cache,
                                           size_t size);

    static void printFunctionEndPThreads(std::ostringstream& cache,
                                         size_t size);

    static void printFileStartOpenMP(std::ostringstream& cache);

    static void printFunctionStartOpenMP(std::ostringstream& cache,
                                         size_t size);

    static void printLoopStartOpenMP(std::ostringstream& cache,
                                     size_t size);

    static void printLoopEndOpenMP(std::ostringstream& cache,
                                   size_t size);

    /**
     *
     */
    inline void startingJob(const std::string& jobName,
                            const JobType& type = JobTypeHolder<>::DEFAULT);

    inline void finishedJob();

    friend class
    ModelLibraryCSourceGen<Base>;

    friend class
    ModelLibraryProcessor<Base>;
};

} // END cg namespace
} // END CppAD namespace

#endif
