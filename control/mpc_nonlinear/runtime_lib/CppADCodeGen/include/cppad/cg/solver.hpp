#ifndef CPPAD_CG_SOLVER_INCLUDED
#define CPPAD_CG_SOLVER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
inline CG<Base> CodeHandler<Base>::solveFor(OperationNode<Base>& expression,
                                            OperationNode<Base>& var) {
    using std::vector;

    // find code in expression
    if (&expression == &var)
        return CG<Base>(var);

    size_t bifurcations = (std::numeric_limits<size_t>::max)(); // so that it is possible to enter the loop

    std::vector<SourceCodePath> paths;
    BidirGraph<Base> foundGraph;
    OperationNode<Base> *root = &expression;

    while (bifurcations > 0) {
        CPPADCG_ASSERT_UNKNOWN(root != nullptr);

        // find possible paths from expression to var
        size_t oldBif = bifurcations;
        bifurcations = 0;
        foundGraph = findPathGraph(*root, var, bifurcations, 50000);
        CPPADCG_ASSERT_UNKNOWN(oldBif > bifurcations);

        if (!foundGraph.contains(var)) {
            std::cerr << "Missing variable " << var << std::endl;
            printExpression(expression, std::cerr);
            throw CGException("The provided variable ", var.getName() != nullptr ? ("(" + *var.getName() + ")") : "", " is not present in the expression");
        }

        // find a bifurcation which does not contain any other bifurcations
        size_t bifPos = 0;
        paths = foundGraph.findSingleBifurcation(*root, var, bifPos);
        if (paths.empty()) {
            throw CGException("The provided variable is not present in the expression");

        } else if (paths.size() == 1) {
            CPPADCG_ASSERT_UNKNOWN(paths[0][0].node == root);
            CPPADCG_ASSERT_UNKNOWN(paths[0].back().node == &var);

            return solveFor(paths[0]);

        } else {
            CPPADCG_ASSERT_UNKNOWN(paths.size() >= 1);
            CPPADCG_ASSERT_UNKNOWN(paths[0].back().node == &var);

            CG<Base> expression2 = collectVariable(*root, paths[0], paths[1], bifPos);
            root = expression2.getOperationNode();
            if (root == nullptr) {
                throw CGException("It is not possible to solve the expression for the requested variable: the variable disappears after symbolic manipulations (e.g., y=x-x).");
            }
        }
    }

    CPPADCG_ASSERT_UNKNOWN(paths.size() == 1);
    return solveFor(paths[0]);
}

template<class Base>
inline CG<Base> CodeHandler<Base>::solveFor(const SourceCodePath& path) {

    CG<Base> rightHs(0.0);

    for (size_t n = 0; n < path.size() - 1; ++n) {
        const OperationPathNode<Base>& pnodeOp = path[n];
        size_t argIndex = path[n].argIndex;
        const std::vector<Argument<Base> >& args = pnodeOp.node->getArguments();

        CGOpCode op = pnodeOp.node->getOperationType();
        switch (op) {
            case CGOpCode::Mul:
            {
                const Argument<Base>& other = args[argIndex == 0 ? 1 : 0];
                rightHs /= CG<Base>(other);
                break;
            }
            case CGOpCode::Div:
                if (argIndex == 0) {
                    const Argument<Base>& other = args[1];
                    rightHs *= CG<Base>(other);
                } else {
                    const Argument<Base>& other = args[0];
                    rightHs = CG<Base>(other) / rightHs;
                }
                break;

            case CGOpCode::UnMinus:
                rightHs *= Base(-1.0);
                break;
            case CGOpCode::Add:
            {
                const Argument<Base>& other = args[argIndex == 0 ? 1 : 0];
                rightHs -= CG<Base>(other);
                break;
            }
            case CGOpCode::Alias:
                // do nothing 
                break;
            case CGOpCode::Sub:
            {
                if (argIndex == 0) {
                    rightHs += CG<Base>(args[1]);
                } else {
                    rightHs = CG<Base>(args[0]) - rightHs;
                }
                break;
            }
            case CGOpCode::Exp:
                rightHs = log(rightHs);
                break;
            case CGOpCode::Log:
                rightHs = exp(rightHs);
                break;
            case CGOpCode::Pow:
            {
                if (argIndex == 0) {
                    // base
                    const Argument<Base>& exponent = args[1];
                    if (exponent.getParameter() != nullptr && *exponent.getParameter() == Base(0.0)) {
                        throw CGException("Invalid zero exponent");
                    } else if (exponent.getParameter() != nullptr && *exponent.getParameter() == Base(1.0)) {
                        continue; // do nothing
                    } else {
                        throw CGException("Unable to invert operation '", op, "'");
                        /*
                        if (exponent.getParameter() != nullptr && *exponent.getParameter() == Base(2.0)) {
                            rightHs = sqrt(rightHs); // TODO: should -sqrt(rightHs) somehow be considered???
                        } else {
                            rightHs = pow(rightHs, Base(1.0) / CG<Base>(exponent));
                        }
                         */
                    }
                } else {
                    // 
                    const Argument<Base>& base = args[0];
                    rightHs = log(rightHs) / log(CG<Base>(base));
                }
                break;
            }
            case CGOpCode::Sqrt:
                rightHs *= rightHs;
                break;
                //case CGAcosOp: // asin(variable)
                //case CGAsinOp: // asin(variable)
                //case Atan: // atan(variable)
            case CGOpCode::Cosh: // cosh(variable)
            {
                rightHs = log(rightHs + sqrt(rightHs * rightHs - Base(1.0))); // asinh
                break;
                //case Cos: //  cos(variable)
            }
            case CGOpCode::Sinh: // sinh(variable)
                rightHs = log(rightHs + sqrt(rightHs * rightHs + Base(1.0))); // asinh
                break;
                //case CGSinOp: //  sin(variable)
            case CGOpCode::Tanh: //  tanh(variable)
                rightHs = Base(0.5) * (log(Base(1.0) + rightHs) - log(Base(1.0) - rightHs)); // atanh
                break;
                //case CGTanOp: //  tan(variable)
            default:
                throw CGException("Unable to invert operation '", op, "'");
        };
    }

    return rightHs;
}

template<class Base>
inline bool CodeHandler<Base>::isSolvable(OperationNode<Base>& expression,
                                          OperationNode<Base>& var) {
    size_t bifurcations = 0;
    BidirGraph<Base> g = findPathGraph(expression, var, bifurcations);

    if(bifurcations == 0) {
        size_t bifIndex = 0;
        auto paths = g.findSingleBifurcation(expression, var, bifIndex);
        if (paths.empty() || paths[0].empty())
            return false;

        return isSolvable(paths[0]);
    } else {
        // TODO: improve this
        //bool v = isCollectableVariableAddSub();
        try {
            solveFor(expression, var);
            return true;
        } catch(const CGException& e) {
            return false;
        }
    }
}

template<class Base>
inline bool CodeHandler<Base>::isSolvable(const SourceCodePath& path) const {
    for (size_t n = 0; n < path.size() - 1; ++n) {
        const OperationPathNode<Base>& pnodeOp = path[n];
        size_t argIndex = path[n].argIndex;
        const std::vector<Argument<Base> >& args = pnodeOp.node->getArguments();

        CGOpCode op = pnodeOp.node->getOperationType();
        switch (op) {
            case CGOpCode::Mul:
            case CGOpCode::Div:
            case CGOpCode::UnMinus:
            case CGOpCode::Add:
            case CGOpCode::Alias:
            case CGOpCode::Sub:
            case CGOpCode::Exp:
            case CGOpCode::Log:
            case CGOpCode::Sqrt:
            case CGOpCode::Cosh: // cosh(variable)
            case CGOpCode::Sinh: // sinh(variable)
            case CGOpCode::Tanh: //  tanh(variable)
                break;
            case CGOpCode::Pow:
            {
                if (argIndex == 0) {
                    // base
                    const Argument<Base>& exponent = args[1];
                    if (exponent.getParameter() != nullptr && *exponent.getParameter() == Base(0.0)) {
                        return false;
                    } else if (exponent.getParameter() != nullptr && *exponent.getParameter() == Base(1.0)) {
                        break;
                    } else {
                        return false;
                    }
                } else {
                    break;
                }
                break;
            }

            default:
                return false;
        };
    }
    return true;
}

} // END cg namespace
} // END CppAD namespace

#endif