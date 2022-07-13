#ifndef CPPAD_CG_EVALUATOR_INCLUDED
#define CPPAD_CG_EVALUATOR_INCLUDED
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

// forward declarations
template<class ScalarIn, class ScalarOut, class ActiveOut, class FinalEvaluatorType>
class EvaluatorOperations;

template<class ScalarIn, class ScalarOut, class ActiveOut, class Operations>
class EvaluatorBase;

/**
 * A base class for evaluators.
 * Evaluators allow to reprocess operations defined in an operation graph
 * for a different set of independent variables and (possibly) data types.
 *
 * Operation implementations (sin(), cos(), ...) should be implemented in a
 * subclass of type FinalEvaluatorType.
 * This allows static polymorphism through curiously recurring template
 * pattern (CRTP). Therefore the default behaviour can be overridden without
 * the use of virtual methods.
 *
 * This class should not be instantiated directly.
 *
 * @todo implement nonrecursive algorithm (so that there will never be any stack limit issues)
 */
template<class ScalarIn, class ScalarOut, class ActiveOut, class FinalEvaluatorType>
class EvaluatorBase {
    friend FinalEvaluatorType;
protected:
    using SourceCodePath = typename CodeHandler<ScalarIn>::SourceCodePath;
protected:
    CodeHandler<ScalarIn>& handler_;
    const ActiveOut* indep_;
    CodeHandlerVector<ScalarIn, std::unique_ptr<ActiveOut>> evals_;
    std::map<size_t, std::vector<ActiveOut>* > evalsArrays_;
    std::map<size_t, std::vector<ActiveOut>* > evalsSparseArrays_;
    bool underEval_;
    size_t depth_;
    SourceCodePath path_;
public:

    /**
     * @param handler The original source code handler
     */
    inline EvaluatorBase(CodeHandler<ScalarIn>& handler) :
        handler_(handler),
        indep_(nullptr),
        evals_(handler),
        underEval_(false),
        depth_(0) { // not really required (but it avoids warnings)
    }

    inline virtual ~EvaluatorBase() {
        clear();
    }

    /**
     * @return true if this Evaluator is currently being used.
     */
    inline bool isUnderEvaluation() {
        return underEval_;
    }

    /**
     * Performs all the operations required to calculate the dependent
     * variables with a (potentially) new data type
     *
     * @param indepNew The new independent variables.
     * @param depOld Dependent variable vector representing the operations that
     *               are going to be executed to determine the new variables
     *               (all variables must belong to the same code handler)
     * @return The dependent variable values
     * @throws CGException on error (such as an unhandled operation type)
     */
    inline std::vector<ActiveOut> evaluate(ArrayView<const ActiveOut> indepNew,
                                           ArrayView<const CG<ScalarIn> > depOld) {
        std::vector<ActiveOut> depNew(depOld.size());

        evaluate(indepNew.data(), indepNew.size(), depNew.data(), depOld.data(), depNew.size());

        return depNew;
    }

    /**
     * Performs all the operations required to calculate the dependent
     * variables with a (potentially) new data type
     *
     * @param indepNew The new independent variables.
     * @param depNew The new dependent variable vector to be computed.
     * @param depOld Dependent variable vector representing the operations that
     *               are going to be executed to determine the new variables
     *               (all variables must belong to the same code handler)
     * @throws CGException on error (such as an different sizes of depNew and
     *         depOld or an unhandled operation type)
     */
    inline void evaluate(ArrayView<const ActiveOut> indepNew,
                         ArrayView<ActiveOut> depNew,
                         ArrayView<const CG<ScalarIn> > depOld) {
        if (depNew.size() != depOld.size()) {
            throw CGException("Dependent array sizes are different.");
        }

        evaluate(indepNew.data(), indepNew.size(), depNew.data(), depOld.data(), depNew.size());
    }

    /**
     * Performs all the operations required to calculate the dependent
     * variables with a (potentially) new data type
     *
     * @param indepNew The new independent variables.
     * @param indepSize The size of the array of independent variables.
     * @param depNew The new dependent variable vector that will be created.
     * @param depOld Dependent variable vector representing the operations that
     *               are going to be executed to determine the new variables
     *               (all variables must belong to the same code handler)
     * @param depSize The size of the array of dependent variables.
     * @throws CGException on error (such as an unhandled operation type)
     */
    inline void evaluate(const ActiveOut* indepNew,
                         size_t indepSize,
                         ActiveOut* depNew,
                         const CG<ScalarIn>* depOld,
                         size_t depSize) {
        if (handler_.getIndependentVariableSize() != indepSize) {
            throw CGException("Invalid independent variable size. Expected ", handler_.getIndependentVariableSize(), " but got ", indepSize, ".");
        }

        CPPADCG_ASSERT_KNOWN(handler_.getIndependentVariableSize() == indepSize, "Invalid size the array of independent variables")

        if (underEval_) {
            throw CGException("The same evaluator cannot be used for simultaneous evaluations. "
                              "Either use a new one or wait for this one to finish its current evaluation.");
        }

        underEval_ = true;

        clear(); // clean-up from any previous call that might have failed
        evals_.adjustSize();

        depth_ = 0;
        path_.clear();

        if(path_.capacity() == 0) {
            path_.reserve(30);
        }

        FinalEvaluatorType& thisOps = static_cast<FinalEvaluatorType&>(*this);

        try {
            thisOps.prepareNewEvaluation();

            indep_ = indepNew;
            thisOps.analyzeOutIndeps(indep_, indepSize);

            for (size_t i = 0; i < depSize; i++) {
                CPPADCG_ASSERT_UNKNOWN(depth_ == 0);
                depNew[i] = evalCG(depOld[i]);
            }

            thisOps.clear(); // clean-up

        } catch (...) {
            underEval_ = false;
            throw;
        }

        underEval_ = false;
    }

protected:

    inline void prepareNewEvaluation() {
        // empty
    }

    /**
     * clean-up
     */
    inline void clear() {
        evals_.clear();

        for (const auto& p : evalsArrays_) {
            delete p.second;
        }
        evalsArrays_.clear();

        for (const auto& p : evalsSparseArrays_) {
            delete p.second;
        }
        evalsSparseArrays_.clear();
    }

    inline void analyzeOutIndeps(const ActiveOut* indep,
                                 size_t n) {
        // empty
    }

    inline ActiveOut evalCG(const CG<ScalarIn>& dep) {
        if (dep.isParameter()) {
            // parameter
            return ActiveOut(dep.getValue());
        } else {
            return evalOperations(*dep.getOperationNode());
        }
    }

    inline ActiveOut evalArg(const std::vector<Argument<ScalarIn> >& args,
                             size_t pos) {
        return evalArg(args[pos], pos);
    }

    inline ActiveOut evalArg(const Argument<ScalarIn>& arg,
                             size_t pos) {
        if (arg.getOperation() != nullptr) {
            path_.back().argIndex = pos;
            ActiveOut a = evalOperations(*arg.getOperation());
            return a;
        } else {
            // parameter
            return ActiveOut(*arg.getParameter());
        }
    }

    inline const ActiveOut& evalOperations(OperationNode<ScalarIn>& node) {
        CPPADCG_ASSERT_KNOWN(node.getHandlerPosition() < handler_.getManagedNodesCount(), "this node is not managed by the code handler")

        // check if this node was previously determined
        if (evals_[node] != nullptr) {
            return *evals_[node];
        }

        // first evaluation of this node
        FinalEvaluatorType& thisOps = static_cast<FinalEvaluatorType&>(*this);

        path_.push_back(OperationPathNode<ScalarIn>(&node, -1));
        depth_++;

        ActiveOut result = thisOps.evalOperation(node);

        // save it for reuse
        ActiveOut* resultPtr = saveEvaluation(node, ActiveOut(result));

        depth_--;
        path_.pop_back();

        return *resultPtr;
    }

    inline ActiveOut* saveEvaluation(const OperationNode<ScalarIn>& node,
                                     ActiveOut&& result) {
        std::unique_ptr<ActiveOut>& resultPtr = evals_[node];
        CPPADCG_ASSERT_UNKNOWN(resultPtr == nullptr); // not supposed to override existing result
        resultPtr.reset(new ActiveOut(std::move(result)));

        ActiveOut* resultPtr2 = resultPtr.get(); // do not use a reference (just in case evals_ is resized)

        FinalEvaluatorType& thisOps = static_cast<FinalEvaluatorType&>(*this);
        thisOps.processActiveOut(node, *resultPtr2);

        return resultPtr2;
    }

    inline std::vector<ActiveOut>& evalArrayCreationOperation(const OperationNode<ScalarIn>& node) {

        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::ArrayCreation, "Invalid array creation operation");
        CPPADCG_ASSERT_KNOWN(node.getHandlerPosition() < handler_.getManagedNodesCount(), "this node is not managed by the code handler")

        // check if this node was previously determined
        auto it = evalsArrays_.find(node.getHandlerPosition());
        if (it != evalsArrays_.end()) {
            return *it->second;
        }

        const std::vector<Argument<ScalarIn> >& args = node.getArguments();
        auto* resultArray = new std::vector<ActiveOut>(args.size());

        // save it for reuse
        evalsArrays_[node.getHandlerPosition()] = resultArray;

        // define its elements
        for (size_t a = 0; a < args.size(); a++) {
            (*resultArray)[a] = evalArg(args, a);
        }

        return *resultArray;
    }

    inline std::vector<ActiveOut>& evalSparseArrayCreationOperation(const OperationNode<ScalarIn>& node) {

        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array creation operation");
        CPPADCG_ASSERT_KNOWN(node.getHandlerPosition() < handler_.getManagedNodesCount(), "this node is not managed by the code handler")

        // check if this node was previously determined
        auto it = evalsSparseArrays_.find(node.getHandlerPosition());
        if (it != evalsSparseArrays_.end()) {
            return *it->second;
        }

        const std::vector<Argument<ScalarIn> >& args = node.getArguments();
        auto* resultArray = new std::vector<ActiveOut>(args.size());

        // save it for reuse
        evalsSparseArrays_[node.getHandlerPosition()] = resultArray;

        // define its elements
        for (size_t a = 0; a < args.size(); a++) {
            (*resultArray)[a] = evalArg(args, a);
        }

        return *resultArray;
    }

};


/**
 * Defines the default operations for evaluators.
 * Evaluators allow to reprocess operations defined in an operation graph
 * for a different set of independent variables and (possibly) data types.
 *
 * This allows static polymorphism through curiously recurring template
 * pattern (CRTP). Therefore the default behaviour can be overridden without
 * the use of virtual methods.
 * This class should not be instantiated directly.
 */
template<class ScalarIn, class ScalarOut, class ActiveOut, class FinalEvaluatorType>
class EvaluatorOperations : public EvaluatorBase<ScalarIn, ScalarOut, ActiveOut, FinalEvaluatorType> {
    /**
     * must be friends with its super classes since there can be a cast to
     * this type due to the curiously recurring template pattern (CRTP)
     */
    friend class EvaluatorBase<ScalarIn, ScalarOut, ActiveOut, FinalEvaluatorType>;
public:
    using Base = EvaluatorBase<ScalarIn, ScalarOut, ActiveOut, FinalEvaluatorType>;
    using NodeIn = OperationNode<ScalarIn>;
    using ArgIn = Argument<ScalarIn>;
public:
    inline EvaluatorOperations(CodeHandler<ScalarIn>& handler):
            Base(handler) {
    }

    virtual ~EvaluatorOperations() { }

protected:

    using Base::evalArg;

    /**
     * Clones a node with the new type.
     * Override this method to add a custom node generation behaviour which
     * does not follow the original operation graph.
     *
     * @param node the original node
     * @return the clone of the original node
     */
    inline ActiveOut evalOperation(OperationNode<ScalarIn>& node) {
        FinalEvaluatorType& thisOps = static_cast<FinalEvaluatorType&>(*this);

        const CGOpCode code = node.getOperationType();
        switch (code) {
            case CGOpCode::Assign:
                return thisOps.evalAssign(node);

            case CGOpCode::Abs: //  abs(variable)
                return thisOps.evalAbs(node);

            case CGOpCode::Acos: // acos(variable)
                return thisOps.evalAcos(node);

            case CGOpCode::Add: //  a + b
                return thisOps.evalAdd(node);

            case CGOpCode::Alias:
                return thisOps.evalAlias(node);

                //case CGArrayCreationOp: // {a, b, c ...}
            case CGOpCode::ArrayElement: // x[i]
                return thisOps.evalArrayElement(node);

            case CGOpCode::Asin: // asin(variable)
                return thisOps.evalAsin(node);

            case CGOpCode::Atan: // atan(variable)
                return thisOps.evalAtan(node);

                //CGAtomicForwardOp
                //CGAtomicReverseOp
            case CGOpCode::ComLt: // return left < right? trueCase: falseCase
                return thisOps.evalCompareLt(node);

            case CGOpCode::ComLe: // return left <= right? trueCase: falseCase
                return thisOps.evalCompareLe(node);

            case CGOpCode::ComEq: // return left == right? trueCase: falseCase
                return thisOps.evalCompareEq(node);

            case CGOpCode::ComGe: // return left >= right? trueCase: falseCase
                return thisOps.evalCompareGe(node);

            case CGOpCode::ComGt: // return left > right? trueCase: falseCase
                return thisOps.evalCompareGt(node);

            case CGOpCode::ComNe: // return left != right? trueCase: falseCase
                return thisOps.evalCompareNe(node);

            case CGOpCode::Cosh: // cosh(variable)
                return thisOps.evalCosh(node);

            case CGOpCode::Cos: //  cos(variable)
                return thisOps.evalCos(node);

            case CGOpCode::Div: // a / b
                return thisOps.evalDiv(node);

            case CGOpCode::Exp: //  exp(variable)
                return thisOps.evalExp(node);

            case CGOpCode::Inv: //                             independent variable
                return thisOps.evalIndependent(node);

            case CGOpCode::Log: //  log(variable)
                return thisOps.evalLog(node);

            case CGOpCode::Mul: // a * b
                return thisOps.evalMul(node);

            case CGOpCode::Pow: //  pow(a,   b)
                return thisOps.evalPow(node);

            case CGOpCode::Pri: //  pow(a,   b)
                return thisOps.evalPrint(node);
                //case PriOp: //  PrintFor(text, parameter or variable, parameter or variable)

            case CGOpCode::Sign: // return (x > 0)? 1.0:((x == 0)? 0.0:-1)
                return thisOps.evalSign(node);

            case CGOpCode::Sinh: // sinh(variable)
                return thisOps.evalSinh(node);

            case CGOpCode::Sin: //  sin(variable)
                return thisOps.evalSin(node);

            case CGOpCode::Sqrt: // sqrt(variable)
                return thisOps.evalSqrt(node);

            case CGOpCode::Sub: //  a - b
                return thisOps.evalSub(node);

            case CGOpCode::Tanh: //  tanh(variable)
                return thisOps.evalTanh(node);

            case CGOpCode::Tan: //  tan(variable)
                return thisOps.evalTan(node);

            case CGOpCode::UnMinus: // -(a)
                return thisOps.evalMinus(node);

            default:
                return thisOps.evalUnsupportedOperation(node);
        }
    }

    inline ActiveOut evalAssign(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for assign()")
        return evalArg(args, 0);
    }

    inline ActiveOut evalAbs(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for abs()")
        return abs(evalArg(args, 0));
    }

    inline ActiveOut evalAcos(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for acos()")
        return acos(evalArg(args, 0));
    }

    inline ActiveOut evalAdd(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 2, "Invalid number of arguments for addition")
        return evalArg(args, 0) + evalArg(args, 1);
    }

    inline ActiveOut evalAlias(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for alias")
        return evalArg(args, 0);
    }

    inline ActiveOut evalArrayElement(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        const std::vector<size_t>& info = node.getInfo();
        CPPADCG_ASSERT_KNOWN(args.size() == 2, "Invalid number of arguments for array element")
        CPPADCG_ASSERT_KNOWN(args[0].getOperation() != nullptr, "Invalid argument for array element");
        CPPADCG_ASSERT_KNOWN(args[1].getOperation() != nullptr, "Invalid argument for array element");
        CPPADCG_ASSERT_KNOWN(info.size() == 1, "Invalid number of information data for array element")
        size_t index = info[0];
        std::vector<ActiveOut>& array = this->evalArrayCreationOperation(*args[0].getOperation()); // array creation

        FinalEvaluatorType& thisOps = static_cast<FinalEvaluatorType&>(*this);
        thisOps.evalAtomicOperation(*args[1].getOperation()); // atomic operation

        return array[index];
    }

    inline ActiveOut evalAsin(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for asin()")
        return asin(evalArg(args, 0));
    }

    inline ActiveOut evalAtan(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for atan()")
        return atan(evalArg(args, 0));
    }

    inline ActiveOut evalCompareLt(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 4, "Invalid number of arguments for CondExpOp(CompareLt, )")
        return CondExpOp(CompareLt, evalArg(args, 0), evalArg(args, 1), evalArg(args, 2), evalArg(args, 3));
    }

    inline ActiveOut evalCompareLe(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 4, "Invalid number of arguments for CondExpOp(CompareLe, )")
        return CondExpOp(CompareLe, evalArg(args, 0), evalArg(args, 1), evalArg(args, 2), evalArg(args, 3));
    }

    inline ActiveOut evalCompareEq(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 4, "Invalid number of arguments for CondExpOp(CompareEq, )")
        return CondExpOp(CompareEq, evalArg(args, 0), evalArg(args, 1), evalArg(args, 2), evalArg(args, 3));
    }

    inline ActiveOut evalCompareGe(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 4, "Invalid number of arguments for CondExpOp(CompareGe, )")
        return CondExpOp(CompareGe, evalArg(args, 0), evalArg(args, 1), evalArg(args, 2), evalArg(args, 3));
    }

    inline ActiveOut evalCompareGt(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 4, "Invalid number of arguments for CondExpOp(CompareGt, )")
        return CondExpOp(CompareGt, evalArg(args, 0), evalArg(args, 1), evalArg(args, 2), evalArg(args, 3));
    }

    inline ActiveOut evalCompareNe(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 4, "Invalid number of arguments for CondExpOp(CompareNe, )")
        return CondExpOp(CompareNe, evalArg(args, 0), evalArg(args, 1), evalArg(args, 2), evalArg(args, 3));
    }

    inline ActiveOut evalCosh(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for cosh()")
        return cosh(evalArg(args, 0));
    }

    inline ActiveOut evalCos(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for cos()")
        return cos(evalArg(args, 0));
    }

    inline ActiveOut evalDiv(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 2, "Invalid number of arguments for division")
        return evalArg(args, 0) / evalArg(args, 1);
    }

    inline ActiveOut evalExp(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for exp()")
        return exp(evalArg(args, 0));
    }

    inline ActiveOut evalIndependent(const NodeIn& node) {
        size_t index = this->handler_.getIndependentVariableIndex(node);
        return this->indep_[index];
    }

    inline ActiveOut evalLog(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for log()")
        return log(evalArg(args, 0));
    }

    inline ActiveOut evalMul(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 2, "Invalid number of arguments for multiplication")
        return evalArg(args, 0) * evalArg(args, 1);
    }

    inline ActiveOut evalPow(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 2, "Invalid number of arguments for pow()")
        return pow(evalArg(args, 0), evalArg(args, 1));
    }

    inline ActiveOut evalPrint(const NodeIn& node) {
        FinalEvaluatorType& thisOps = static_cast<FinalEvaluatorType&>(*this);
        return thisOps.evalUnsupportedOperation(node);
    }

    //case PriOp: //  PrintFor(text, parameter or variable, parameter or variable)
    inline ActiveOut evalSign(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for sign()")
        return sign(evalArg(args, 0));
    }

    inline ActiveOut evalSinh(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for sinh()")
        return sinh(evalArg(args, 0));
    }

    inline ActiveOut evalSin(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for sin()")
        return sin(evalArg(args, 0));
    }

    inline ActiveOut evalSqrt(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for sqrt()")
        return sqrt(evalArg(args, 0));
    }

    inline ActiveOut evalSub(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 2, "Invalid number of arguments for subtraction")
        return evalArg(args, 0) - evalArg(args, 1);
    }

    inline ActiveOut evalTanh(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for tanh()")
        return tanh(evalArg(args, 0));
    }

    inline ActiveOut evalTan(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for tan()")
        return tan(evalArg(args, 0));
    }

    inline ActiveOut evalMinus(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for unary minus")
        return -evalArg(args, 0);
    }

    inline ActiveOut evalUnsupportedOperation(const NodeIn& node) {
        throw CGException("Unknown operation code '", node.getOperationType(), "'");
    }

    inline void evalAtomicOperation(const NodeIn& node) {
        throw CGException("Evaluator is unable to handle atomic functions for these variable types");
    }

    inline void processActiveOut(const NodeIn& node,
                                 ActiveOut& a) {
    }
};

/**
 * An evaluator allows to reprocess operations defined in an operation graph
 * for a different set of independent variables and (possibly) data types.
 */
template<class ScalarIn, class ScalarOut, class ActiveOut = CppAD::AD<ScalarOut> >
class Evaluator : public EvaluatorOperations<ScalarIn, ScalarOut, ActiveOut, Evaluator<ScalarIn, ScalarOut, ActiveOut> > {
public:
    using Base = EvaluatorOperations<ScalarIn, ScalarOut, ActiveOut, Evaluator<ScalarIn, ScalarOut, ActiveOut> >;
public:

    inline Evaluator(CodeHandler<ScalarIn>& handler) :
            Base(handler) {
    }

    inline virtual ~Evaluator() = default;
};

} // END cg namespace
} // END CppAD namespace

#endif
