#ifndef CPPAD_CG_EVALUATOR_AD_INCLUDED
#define CPPAD_CG_EVALUATOR_AD_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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
 * Specialization of class Evaluator for an output active type of AD<>
 * This class should not be instantiated directly.
 */
template<class ScalarIn, class ScalarOut, class FinalEvaluatorType>
class EvaluatorAD : public EvaluatorOperations<ScalarIn, ScalarOut, CppAD::AD<ScalarOut>, FinalEvaluatorType> {
    /**
     * must be friends with one of its super classes since there is a cast to
     * this type due to the curiously recurring template pattern (CRTP)
     */
    friend EvaluatorOperations<ScalarIn, ScalarOut, CppAD::AD<ScalarOut>, FinalEvaluatorType>;
    friend EvaluatorBase<ScalarIn, ScalarOut, CppAD::AD<ScalarOut>, FinalEvaluatorType>;
public:
    using ActiveOut = CppAD::AD<ScalarOut>;
    using NodeIn = OperationNode<ScalarIn>;
    using ArgIn = Argument<ScalarIn>;
    using Super = EvaluatorOperations<ScalarIn, ScalarOut, CppAD::AD<ScalarOut>, FinalEvaluatorType>;
protected:
    using Super::handler_;
    using Super::evalArrayCreationOperation;
protected:
    std::set<NodeIn*> evalsAtomic_;
    std::map<size_t, CppAD::atomic_base<ScalarOut>* > atomicFunctions_;
    /**
     * Whether or not the nodes with an operation type 'Pri' are printed out
     * during the evaluation.
     */
    bool printOutPriOperations_;
public:

    inline EvaluatorAD(CodeHandler<ScalarIn>& handler) :
        Super(handler),
        printOutPriOperations_(true) {
    }

    inline virtual ~EvaluatorAD() = default;

    /**
     * Defines whether or not to print out the nodes with an operation type
     * 'Pri' during the evaluation.
     */
    inline void setPrintOutPrintOperations(bool print) {
        printOutPriOperations_ = print;
    }

    /**
     * Whether or not the nodes with an operation type 'Pri' are printed out
     * during the evaluation.
     */
    inline bool isPrintOutPrintOperations() const {
        return printOutPriOperations_;
    }

    /**
     * Provides an atomic function.
     *
     * @param id The atomic function ID
     * @param atomic The atomic function
     * @return True if an atomic function with the same ID was already
     *         defined, false otherwise.
     */
    virtual bool addAtomicFunction(size_t id, atomic_base<ScalarOut>& atomic) {
        bool exists = atomicFunctions_.find(id) != atomicFunctions_.end();
        atomicFunctions_[id] = &atomic;
        return exists;
    }

    virtual void addAtomicFunctions(const std::map<size_t, atomic_base<ScalarOut>* >& atomics) {
        for (const auto& it : atomics) {
            atomic_base<ScalarOut>* atomic = it.second;
            if (atomic != nullptr) {
                atomicFunctions_[it.first] = atomic;
            }
        }
    }

    /**
     * Provides the number of atomic function evaluations.
     * The same function can be considered more than once.
     *
     * @return the number of atomic functions evaluations.
     */
    size_t getNumberOfEvaluatedAtomics() const {
        return evalsAtomic_.size();
    }

protected:

    /**
     * @note overrides the default prepareNewEvaluation() even though this method
     *        is not virtual (hides a method in EvaluatorBase)
     */
    inline void prepareNewEvaluation() {
        /**
         * Do not place this in clear() so that it is possible to determine
         * the number of atomic function evaluations after the evaluation has
         * ended.
         */
        Super::prepareNewEvaluation();

        evalsAtomic_.clear();
    }

    /**
     * @throws CGException on an internal evaluation error
     *
     * @note overrides the default evalAtomicOperation() even though this
     *       method is not virtual (hides a method in EvaluatorOperations)
     */
    inline void evalAtomicOperation(NodeIn& node) {

        if (evalsAtomic_.find(&node) != evalsAtomic_.end()) {
            return;
        }

        if (node.getOperationType() != CGOpCode::AtomicForward) {
            throw CGException("Evaluator can only handle zero forward mode for atomic functions");
        }

        const std::vector<size_t>& info = node.getInfo();
        const std::vector<Argument<ScalarIn> >& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 2, "Invalid number of arguments for atomic forward mode")
        CPPADCG_ASSERT_KNOWN(info.size() == 3, "Invalid number of information data for atomic forward mode")

        // find the atomic function
        size_t id = info[0];
        typename std::map<size_t, atomic_base<ScalarOut>* >::const_iterator itaf = atomicFunctions_.find(id);
        atomic_base<ScalarOut>* atomicFunction = nullptr;
        if (itaf != atomicFunctions_.end()) {
            atomicFunction = itaf->second;
        }

        if (atomicFunction == nullptr) {
            std::stringstream ss;
            ss << "No atomic function defined in the evaluator for ";
            const std::string & atomName = handler_.getAtomicFunctionName(id);
            if (!atomName.empty()) {
                ss << "'" << atomName << "'";
            } else
                ss << "id '" << id << "'";
            throw CGException(ss.str());
        }

        size_t p = info[2];
        if (p != 0) {
            throw CGException("Evaluator can only handle zero forward mode for atomic functions");
        }
        const std::vector<ActiveOut>& ax = evalArrayCreationOperation(*args[0].getOperation());
        std::vector<ActiveOut>& ay = evalArrayCreationOperation(*args[1].getOperation());

        (*atomicFunction)(ax, ay);

        evalsAtomic_.insert(&node);
    }

    /**
     * @note overrides the default evalPrint() even though this method
     *        is not virtual (hides a method in EvaluatorOperations)
     */
    inline ActiveOut evalPrint(const NodeIn& node) {
        const std::vector<ArgIn>& args = node.getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() == 1, "Invalid number of arguments for print()")
        ActiveOut out(this->evalArg(args, 0));

        const auto& nodePri = static_cast<const PrintOperationNode<ScalarIn>&>(node);
        if (printOutPriOperations_) {
            std::cout << nodePri.getBeforeString() << out << nodePri.getAfterString();
        }

        CppAD::PrintFor(ActiveOut(0), nodePri.getBeforeString().c_str(), out, nodePri.getAfterString().c_str());

        return out;
    }

};

/**
 * Specialization of Evaluator for an output active type of AD<>
 */
template<class ScalarIn, class ScalarOut>
class Evaluator<ScalarIn, ScalarOut, CppAD::AD<ScalarOut> > : public EvaluatorAD<ScalarIn, ScalarOut, Evaluator<ScalarIn, ScalarOut, CppAD::AD<ScalarOut> > > {
public:
    using ActiveOut = CppAD::AD<ScalarOut>;
    using Super = EvaluatorAD<ScalarIn, ScalarOut, Evaluator<ScalarIn, ScalarOut, CppAD::AD<ScalarOut> > >;
public:

    inline Evaluator(CodeHandler<ScalarIn>& handler) :
        Super(handler) {
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
