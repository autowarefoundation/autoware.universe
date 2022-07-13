#ifndef CPPAD_CG_COLLECT_VARIABLE_INCLUDED
#define CPPAD_CG_COLLECT_VARIABLE_INCLUDED
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

#include <cppad/cg/evaluator/evaluator_solve.hpp>
#include <cppad/cg/lang/dot/dot.hpp>

namespace CppAD {
namespace cg {

template<class Base>
inline CG<Base> CodeHandler<Base>::collectVariable(OperationNode<Base>& expression,
                                                   const SourceCodePath& path1,
                                                   const SourceCodePath& path2,
                                                   size_t lastCommon) {
    CPPADCG_ASSERT_UNKNOWN(lastCommon >= 0);
    CPPADCG_ASSERT_UNKNOWN(path1.size() > lastCommon);
    CPPADCG_ASSERT_UNKNOWN(path2.size() > lastCommon);

#ifndef NDEBUG
    /**
     * Check common path
     */
    for(size_t i = 0;i < lastCommon; ++i) {
        // compare with the first path
        CPPADCG_ASSERT_UNKNOWN(path2[i] == path1[i]);
    }
#endif

    //std::cout << "common sub-expression (before):\n";
    //printExpression(*path1[lastCommon].node);

    SourceCodePath leftPath(path1.begin() + lastCommon, path1.end());
    SourceCodePath rightPath(path2.begin() + lastCommon, path2.end());

    CG<Base> subExpression = collectVariableAddSub(leftPath, rightPath);

    //std::cout << "common sub-expression (after):\n";
    //printExpression(subExpression);

    /**
     * Replace the new expression
     */
    std::vector<CG<Base>*> replace(lastCommon + 1, nullptr);
    //replace.back() = &subExpression;
    std::vector<const std::vector<CG<Base>*>*> replaceOnPath{&replace};

    SourceCodePath initPath(path1.begin(), path1.begin() + lastCommon + 1);
    std::vector<const SourceCodePath*> initPaths{&initPath};

    //std::map<OperationNode<Base>*, CG<Base> > replacementNodes;
    //replacementNodes[path1[lastCommon].node] = subExpression; // it might be used in more places within the model

    // use same independent variables
    std::vector<CG<Base>> indep(this->_independentVariables.size());
    for (size_t i = 0; i < indep.size(); ++i)
        indep[i] = CG<Base>(*this->_independentVariables[i]);

    CG<Base> expressionOrig(expression);
    CG<Base> expression2;

    size_t bifurcations = 0;
    BidirGraph<Base> graphToCommon = findPathGraph(expression, *path1[lastCommon].node, bifurcations);
    std::map<const PathNodeEdges<Base>*, CG<Base>> replacementNodes;
    replacementNodes[&graphToCommon[*path1[lastCommon].node]] = subExpression;

    EvaluatorCloneSolve<Base> e(*this, graphToCommon, replacementNodes);
    e.evaluate(indep.data(), indep.size(),
               &expression2, &expressionOrig, 1);


    //std::cout << "expression before replacement:\n";
    //printExpression(expressionOrig);
    //printDotExpression(expressionOrig);

    //std::cout << "expression after replacement:\n";
    //printExpression(expression2);
    // TODO:
    //  exp(x)*exp(x+1) = exp(x+x+1)
    //  pow(2, x)*pow(2, x+1) = pow(2, x+x+1)
    //  1 / (2*x) + 2 / (3*x) = (1*3 + 2*2) / (6*x)

    return expression2;
}

template<class Base>
inline CG<Base> CodeHandler<Base>::collectVariableAddSub(const SourceCodePath& pathLeft,
                                                         const SourceCodePath& pathRight) {
    /**
     * Argument validation
     */
    if(pathLeft[0].node == nullptr) {
        throw CGException("Invalid path!");
    } else if(pathLeft[0].node != pathRight[0].node) {
        throw CGException("The first element in each path must be the same.");
    }

    auto& expression = *pathLeft[0].node;

    auto initCommonOp = expression.getOperationType();
    if (initCommonOp != CGOpCode::Add &&
        initCommonOp != CGOpCode::Sub) {
        throw CGException("Invalid path! It must start with either an addition or subtraction.");
    }

    if(pathLeft.back().node == nullptr || pathRight.back().node == nullptr)
        throw CGException("Invalid path! It must end with a non null node.");

    if(pathLeft.back().node != pathRight.back().node)
        throw CGException("Invalid paths! They must end with the same node.");

    OperationNode<Base>& var = *pathLeft.back().node;

    /**
     * Check operations after the divergence
     */
    isCollectableVariableAddSub(pathLeft, pathRight, true);

    /**
     *
     */
    CG<Base> zero(0);
    std::array<std::vector<CG<Base>*>, 2> replace; // replacements for the existing operations

    std::vector<const SourceCodePath*> paths{&pathLeft, &pathRight};

    CG<Base> cSum;

    for (size_t j = 0; j < paths.size(); ++j) {
        const auto& p = *paths[j];
        replace[j] = std::vector<CG<Base>*>(p.size(), nullptr);
        replace[j].back() = &zero; // the last node is always removed (its the variable we want to extract)

        /**
         * find the coefficient which will multiply the variable
         */
        CG<Base> c = 1;
        if (initCommonOp == CGOpCode::Sub && j == 1) {
            c = -1;
        }

        for (size_t i = 1; i < p.size() - 1; ++i) { // the last node is what we want to extract
            const auto* node = p[i].node;
            if (node == nullptr) {
                throw CGException("Failed to combine multiple occurrences of a variable into one expression");
            }

            auto op = node->getOperationType();

            if (op == CGOpCode::Add) {
                continue;

            } else if (op == CGOpCode::Sub) {
                if (p[i - 1].argIndex == 1)
                    c = -c;
                else
                    continue;

            } else if (op == CGOpCode::UnMinus) {
                c = -c;

            } else if (op == CGOpCode::Mul) {
                CPPADCG_ASSERT_UNKNOWN(p[i].argIndex == 0 || p[i].argIndex == 1);
                const auto& pArgs = node->getArguments();
                c *= (p[i].argIndex == 0) ? pArgs[1] : pArgs[0];

            } else if (op == CGOpCode::Div) {
                CPPADCG_ASSERT_UNKNOWN(p[i].argIndex == 0);
                c /= CG<Base>(node->getArguments()[1]);

            } else if (op == CGOpCode::Alias) {
                continue;

            } else {
                // should never get here
                CPPADCG_ASSERT_UNKNOWN(false);
            }

        }

        //
        cSum += c;

        for (size_t i1 = p.size() - 1; i1 > 0; --i1) {
            size_t i = i1 - 1;
            if (p[i].node->getOperationType() == CGOpCode::Mul ||
                p[i].node->getOperationType() == CGOpCode::UnMinus ||
                p[i].node->getOperationType() == CGOpCode::Div ||
                p[i].node->getOperationType() == CGOpCode::Alias) {
                replace[j][i] = &zero;
            } else {
                break;
            }
        }
    }

    const auto& replaceLeft = replace[0];
    const auto& replaceRight = replace[1];
    bool keepCommon = replaceLeft[1] == nullptr || replaceRight[1] == nullptr;

    /**
     * Clone operations in expression without var
     */
    CG<Base> expression2;
    if (keepCommon) {
        std::vector<const std::vector<CG<Base>*>*> replaceOnPath{&replaceLeft, &replaceRight};
        EvaluatorCloneSolve<Base> e(*this, paths, replaceOnPath);

        std::vector<CG<Base>> indep(this->_independentVariables.size());
        for (size_t i = 0; i < indep.size(); ++i)
            indep[i] = CG<Base>(*this->_independentVariables[i]);

        CG<Base> expressionOrig(expression);

        e.evaluate(indep.data(), indep.size(),
                   &expression2, &expressionOrig, 1);
    }

    /**
     * add term for variable
     */
    CG<Base> v(var);
    return cSum * v + expression2;
}

template<class Base>
inline bool CodeHandler<Base>::isCollectableVariableAddSub(const SourceCodePath& pathLeft,
                                                           const SourceCodePath& pathRight,
                                                           bool throwEx) {
    /**
     * Check operations after the divergence
     */
    std::vector<const SourceCodePath*> paths{&pathLeft, &pathRight};
    for (const SourceCodePath* p: paths) {
        for (size_t i = 1; i < p->size() - 1; ++i) { // the last node does not have to be +, -, or *
            const auto* node = (*p)[i].node;

            if (node == nullptr) {
                if (throwEx)
                    throw CGException("Failed to combine multiple occurrences of a variable into one expression");
                return false;
            }

            auto op = node->getOperationType();
            if (op != CGOpCode::Add &&
                op != CGOpCode::Sub &&
                op != CGOpCode::Mul &&
                op != CGOpCode::UnMinus &&
                op != CGOpCode::Alias) {

                if (op == CGOpCode::Div) {
                    if ((*p)[i].argIndex != 0) {
                        if (throwEx)
                            throw CGException("Unable to combine operations which are present in denominators (not implemented yet).");
                        return false;
                    } else {
                        continue;
                    }
                }

                if (throwEx)
                    throw CGException("Failed to combine multiple occurrences of a variable into one expression."
                                      " Unable to combine operations diverging at a '", op, "' operation.");
                return false;
            }
        }
    }

    return true;
}

} // END cg namespace
} // END CppAD namespace

#endif