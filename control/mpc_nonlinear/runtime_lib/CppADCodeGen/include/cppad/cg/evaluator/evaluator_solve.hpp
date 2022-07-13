#ifndef CPPAD_CG_EVALUATOR_SOLVE_INCLUDED
#define CPPAD_CG_EVALUATOR_SOLVE_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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
 * Specialization of EvaluatorCG which can replace some operations.
 * It only clones some of the nodes.
 * It is used by the symbolic solver.
 */
template<class Scalar>
class EvaluatorCloneSolve : public EvaluatorCG<Scalar, Scalar, EvaluatorCloneSolve<Scalar>> {
    /**
     * must be friends with one of its super classes since there is a cast to
     * this type due to the curiously recurring template pattern (CRTP)
     */
    using FinalEvaluatorType = EvaluatorCloneSolve<Scalar>;
    friend EvaluatorBase<Scalar, Scalar, CG<Scalar>, FinalEvaluatorType>;
public:
    using ActiveOut = CG<Scalar>;
    using SourceCodePath = typename CodeHandler<Scalar>::SourceCodePath;
protected:
    using Super = EvaluatorCG<Scalar, Scalar, FinalEvaluatorType>;
private:
    /**
     * the operation paths which should be cloned or replaced with values in
     * replacement_
     */
    const std::vector<const SourceCodePath*>* paths_;
    /**
     * replacements for the operations along the paths
     * (a null means that the original should be cloned)
     */
    const std::vector<const std::vector<CG<Scalar>*>*>* replaceOnPath_;
    /**
     * the operation paths which should be cloned or replaced with values in
     * replaceOnGraph_
     */
    const BidirGraph<Scalar>* pathGraph_;
    /**
     * replacements for the operations along the paths
     */
    const std::map<const PathNodeEdges<Scalar>*, CG<Scalar>>* replaceOnGraph_;
    /**
     * operations which should be cloned
     */
    const std::set<const OperationNode<Scalar>*>* clone_;
    /**
     * replacements for the operations along the paths
     */
    const std::map<const OperationPathNode<Scalar>, CG<Scalar>>* replaceArgument_;
public:

    /**
     * Creates a new evaluator.
     *
     * @param handler
     * @param paths operation nodes in a path which should be cloned
     *              (there shouldn't be multiple usages of these nodes)
     * @param replaceOnPath replacements for the operations along the paths
     *                      (a null means that the original should be cloned)
     */
    inline EvaluatorCloneSolve(CodeHandler<Scalar>& handler,
                               const std::vector<const SourceCodePath*>& paths,
                               const std::vector<const std::vector<CG<Scalar>*>*>& replaceOnPath) :
            Super(handler),
            paths_(&paths),
            replaceOnPath_(&replaceOnPath),
            pathGraph_(nullptr),
            replaceOnGraph_(nullptr),
            clone_(nullptr),
            replaceArgument_(nullptr) {
        CPPADCG_ASSERT_UNKNOWN(paths_->size() == replaceOnPath_->size());
#ifndef NDEBUG
        for (size_t i = 0; i < paths.size(); ++i) {
            CPPADCG_ASSERT_UNKNOWN(paths[i]->size() == replaceOnPath[i]->size());
        }
#endif
    }

    /**
     * Creates a new evaluator.
     *
     * @param handler
     * @param pathGraph the operation paths which should be cloned
     * @param replaceOnGraph replacements for the operations along the graph
     */
    inline EvaluatorCloneSolve(CodeHandler<Scalar>& handler,
                               const BidirGraph<Scalar>& pathGraph,
                               const std::map<const PathNodeEdges<Scalar>*, CG<Scalar> >& replaceOnGraph) :
            Super(handler),
            paths_(nullptr),
            replaceOnPath_(nullptr),
            pathGraph_(&pathGraph),
            replaceOnGraph_(&replaceOnGraph),
            clone_(nullptr),
            replaceArgument_(nullptr) {
    }

    /**
     * Creates a new evaluator.
     *
     * @param handler
     * @param clone operations which should be cloned
     * @param replaceArgument replacements for the operations along the paths
     */
    inline EvaluatorCloneSolve(CodeHandler<Scalar>& handler,
                               const std::set<const OperationNode<Scalar>*>& clone,
                               const std::map<const OperationPathNode<Scalar>, CG<Scalar>>& replaceArgument) :
        Super(handler),
        paths_(nullptr),
        replaceOnPath_(nullptr),
        pathGraph_(nullptr),
        replaceOnGraph_(nullptr),
        clone_(&clone),
        replaceArgument_(&replaceArgument) {
    }

protected:

    /**
     * @note overrides the default evalOperation() even though this method
     *        is not virtual (hides a method in EvaluatorOperations)
     */
    inline ActiveOut evalOperation(OperationNode<Scalar>& node) {
        CPPADCG_ASSERT_UNKNOWN(this->depth_ > 0);

        if(paths_ != nullptr) {
            const auto& paths = *paths_;
            for (size_t i = 0; i < paths.size(); ++i) {
                size_t d = this->depth_ - 1;
                if (isOnPath(*paths[i])) {
                    // in one of the paths

                    auto* r = (*(*replaceOnPath_)[i])[d];
                    if (r != nullptr) {
                        return *r;
                    } else {
                        return Super::evalOperation(node);
                    }
                }
            }
        }

        if(pathGraph_ != nullptr) {
            const PathNodeEdges<Scalar>* egdes = pathGraph_->find(node);
            if (egdes != nullptr) {
                auto it = replaceOnGraph_->find(egdes);
                if (it != replaceOnGraph_->end()) {
                    return it->second;
                } else {
                    return Super::evalOperation(node);
                }
            }
        }

        if (clone_ != nullptr) {
            if (clone_->find(&node) != clone_->end()) {
                return Super::evalOperation(node);
            }
        }

        if (replaceArgument_ != nullptr) {
            size_t d = this->depth_ - 1;
            if (d > 0) {
                auto it = replaceArgument_->find(this->path_[d - 1]);
                if (it != replaceArgument_->end()) {
                    return it->second;
                }
            }
        }

        return CG<Scalar>(node); // use original
    }

private:
    inline bool isOnPath(const SourceCodePath& path) const {
        size_t d = this->depth_ - 1;

        if (d >= path.size())
            return false;

        if (this->path_[d].node != path[d].node) // compare only the node
            return false;

        if (d > 0) {
            for (size_t j = 0; j < d; ++j) {
                if (this->path_[j] != path[j]) { // compare node and argument index
                    return false;
                }
            }
        }

        return true;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
