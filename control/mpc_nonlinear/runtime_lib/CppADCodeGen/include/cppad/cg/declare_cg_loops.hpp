#ifndef CPPAD_CG_DECLARE_CG_LOOPS_INCLUDED
#define CPPAD_CG_DECLARE_CG_LOOPS_INCLUDED
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

// forward declarations
namespace CppAD {
namespace cg {

template<class Base>
class vector;

namespace loops {

using SizeN1stIt = std::pair<size_t, size_t>;

using pairss = std::pair<size_t, size_t>;

class JacobianWithLoopsRowInfo;

class HessianElement;

template<class Base>
class LoopNonIndexedLocator;

template<class Base>
class IfBranchInfo;

template <class Base>
class IfElseInfo;

template<class Base>
class JacobianTermContrib;

template<class Base>
class JacobianColGroup;

template<class Base>
class HessianWithLoopsInfo;

template<class Base>
class HessianWithLoopsEquationGroupInfo;

template<class Base>
class HessianRowGroup;

class ArrayGroup;

template<class Base>
inline std::vector<CG<Base> > createIndexedIndependents(CodeHandler<Base>& handler,
                                                        LoopModel<Base>& loop,
                                                        IndexOperationNode<Base>& iterationIndexOp);

template<class Base>
inline std::vector<CG<Base> > createLoopIndependentVector(CodeHandler<Base>& handler,
                                                          LoopModel<Base>& loop,
                                                          const std::vector<CG<Base> >& indexedIndeps,
                                                          const std::vector<CG<Base> >& nonIndexedIndeps,
                                                          const std::vector<CG<Base> >& nonIndexedTmps);

template<class Base>
inline std::vector<CG<Base> > createLoopDependentVector(CodeHandler<Base>& handler,
                                                        LoopModel<Base>& loop,
                                                        IndexOperationNode<Base>& iterationIndexOp);

template<class Base>
inline CG<Base> createLoopDependentFunctionResult(CodeHandler<Base>& handler,
                                                  size_t i, const CG<Base>& val, IndexPattern* ip,
                                                  IndexOperationNode<Base>& iterationIndexOp);

template<class Base>
inline LoopEndOperationNode<Base>* createLoopEnd(CodeHandler<Base>& handler,
                                                 LoopStartOperationNode<Base>& loopStart,
                                                 const std::vector<std::pair<CG<Base>, IndexPattern*> >& indexedLoopResults,
                                                 const std::set<IndexOperationNode<Base>*>& indexesOps,
                                                 size_t assignOrAdd);

template<class Base>
inline void moveNonIndexedOutsideLoop(LoopStartOperationNode<Base>& loopStart,
                                      LoopEndOperationNode<Base>& loopEnd);

template<class Base>
inline bool findNonIndexedNodes(OperationNode<Base>& node,
                                std::set<OperationNode<Base>*>& nonIndexed,
                                const OperationNode<Base>& loopIndex);

template<class Base>
inline IfElseInfo<Base>* findExistingIfElse(std::vector<IfElseInfo<Base> >& ifElses,
                                            const std::map<SizeN1stIt, std::pair<size_t, std::set<size_t> > >& first2Iterations);

inline std::vector<size_t> createIndexConditionExpression(const std::set<size_t>& iterations,
                                                          const std::set<size_t>& usedIter,
                                                          size_t maxIter);

template<class Base>
inline OperationNode<Base>* createIndexConditionExpressionOp(CodeHandler<Base>& handler,
                                                             const std::set<size_t>& iterations,
                                                             const std::set<size_t>& usedIter,
                                                             size_t maxIter,
                                                             IndexOperationNode<Base>& iterationIndexOp);

template<class Base>
inline void determineForRevUsagePatterns(const std::map<LoopModel<Base>*, std::map<size_t, std::map<size_t, std::set<size_t> > > >& loopGroups,
                                         const std::map<size_t, std::vector<std::set<size_t> > >& userElLocation,
                                         const std::map<size_t, bool>& ordered,
                                         std::map<size_t, std::map<LoopModel<Base>*, std::map<size_t, ArrayGroup*> > >& loopCalls,
                                         SmartVectorPointer<ArrayGroup>& garbage);

template<class Base>
void generateFunctionDeclarationSourceLoopForRev(std::ostringstream& cache,
                                                 LanguageC<Base>& langC,
                                                 const std::string& modelName,
                                                 const std::string& keyName,
                                                 const std::map<LoopModel<Base>*, std::map<size_t, std::map<size_t, std::set<size_t> > > >& _loopRev2Groups,
                                                 void (*generateFunctionNameLoopRev2)(std::ostringstream& cache, const std::string& modelName, const LoopModel<Base>& loop, size_t g));

template<class Base>
inline void generateLoopForJacHes(ADFun<CG<Base> >& fun,
                                  const std::vector<CG<Base> >& x,
                                  const std::vector<std::vector<CG<Base> > >& vw,
                                  std::vector<CG<Base> >& y,
                                  const std::vector<std::set<size_t> >& jacSparsity,
                                  const std::vector<std::set<size_t> >& jacEvalSparsity,
                                  std::vector<std::map<size_t, CG<Base> > >& jac,
                                  const std::vector<std::set<size_t> >& hesSparsity,
                                  const std::vector<std::set<size_t> >& hesEvalSparsity,
                                  std::vector<std::map<size_t, std::map<size_t, CG<Base> > > >& vhess,
                                  bool constainsAtomics);

} // END loops namespace

} // END cg namespace
} // END CppAD namespace

#endif
