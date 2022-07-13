#ifndef CPPAD_CG_ATOMIC_DEPENDENCY_LOCATOR_INCLUDED
#define CPPAD_CG_ATOMIC_DEPENDENCY_LOCATOR_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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
 * Utility class that holds information on how atomic functions are used
 */
template<class Base>
class AtomicUseInfo {
public:
    /**
     * the atomic function
     */
    CGAbstractAtomicFun<Base>* atom;
    /**
     * pairs of independent and dependent sizes
     */
    std::set<std::pair<size_t, size_t>> sizes;
    /**
     * the outer independent variable indexes which affect a call of that atomic function
     */
    std::set<size_t> outerIndeps;
public:
    inline AtomicUseInfo() :
            atom(nullptr) {
    }
};

/**
 * Finds atomic functions in a CppAD tape and collects some information on how
 * they are used.
 */
template<class Base>
class AtomicDependencyLocator {
private:
    ADFun<CG<Base> >& fun_;
    std::map<size_t, AtomicUseInfo<Base>> atomicInfo_;
    std::map<OperationNode<Base>*, std::set<size_t> > indeps_;
    CodeHandler<Base> handler_;
public:

    inline AtomicDependencyLocator(ADFun<CG<Base> >& fun) :
        fun_(fun) {
    }

    inline const std::map<size_t, AtomicUseInfo<Base>>& findAtomicsUsage() {
        if (!atomicInfo_.empty()) {
            return atomicInfo_;
        }

        size_t m = fun_.Range();
        size_t n = fun_.Domain();

        std::vector<CG<Base> > x(n);
        handler_.makeVariables(x);

        // make sure the position in the code handler is the same as the independent index
        assert(x.size() == 0 || (x[0].getOperationNode()->getHandlerPosition() == 0 && x[x.size() - 1].getOperationNode()->getHandlerPosition() == x.size() - 1));

        std::vector<CG<Base> > dep = fun_.Forward(0, x);

        for (size_t i = 0; i < m; i++) {
            findAtomicsUsage(dep[i].getOperationNode());
        }

        const auto& regAtomics = handler_.getAtomicFunctions();
        for (auto& pair: atomicInfo_) {
            size_t id = pair.first;

            pair.second.atom = regAtomics.at(id);
        }

        return atomicInfo_;
    }

private:

    inline std::set<size_t> findAtomicsUsage(OperationNode<Base>* node) {
        if (node == nullptr)
            return std::set<size_t>();

        CGOpCode op = node->getOperationType();
        if (op == CGOpCode::Inv) {
            std::set<size_t> indeps;
            // particular case where the position in the code handler is the same as the independent index
            indeps.insert(node->getHandlerPosition());
            return indeps;
        }

        if (handler_.isVisited(*node)) {
            // been here before
            return indeps_.at(node);
        }

        handler_.markVisited(*node);

        std::set<size_t> indeps;
        const std::vector<Argument<Base> >& args = node->getArguments();
        for (size_t a = 0; a < args.size(); a++) {
            std::set<size_t> aindeps = findAtomicsUsage(args[a].getOperation());
            indeps.insert(aindeps.begin(), aindeps.end());
        }
        indeps_[node] = indeps;

        if (op == CGOpCode::AtomicForward) {
            CPPADCG_ASSERT_UNKNOWN(node->getInfo().size() > 1);
            CPPADCG_ASSERT_UNKNOWN(node->getArguments().size() > 1);
            size_t id = node->getInfo()[0];

#ifndef NDEBUG
            size_t p = node->getInfo()[2];
            CPPADCG_ASSERT_UNKNOWN(p == 0);
#endif

            OperationNode<Base>* tx = node->getArguments()[0].getOperation();
            OperationNode<Base>* ty = node->getArguments()[1].getOperation();

            CPPADCG_ASSERT_UNKNOWN(tx != nullptr && tx->getOperationType() == CGOpCode::ArrayCreation);
            CPPADCG_ASSERT_UNKNOWN(ty != nullptr && ty->getOperationType() == CGOpCode::ArrayCreation);

            auto& info = atomicInfo_[id];
            info.outerIndeps.insert(indeps.begin(), indeps.end());
            info.sizes.insert(std::pair<size_t, size_t>(tx->getArguments().size(),
                                                        ty->getArguments().size()));
        }

        return indeps;
    }
};

} // END cg namespace
} // END CppAD namespace

#endif