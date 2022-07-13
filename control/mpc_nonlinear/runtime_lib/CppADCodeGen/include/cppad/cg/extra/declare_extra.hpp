#ifndef CPPAD_CG_DECLARE_EXTRA_INCLUDED
#define CPPAD_CG_DECLARE_EXTRA_INCLUDED
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

#include <cppad/local/define.hpp>
#include <cppad/local/cppad_assert.hpp>
#include <cppad/local/base_cond_exp.hpp>

// forward declarations
namespace CppAD {
namespace cg {

/***********************************************************************
 * Combined Jacobian and Hessian evaluation
 **********************************************************************/

class SparseForjacHessianWorkJac;
class SparseForjacHessianWorkHes;
class SparseForjacHessianWork;

template<class Base, class VectorBase, class VectorSet, class VectorSize>
size_t sparseForJacHessian(ADFun<Base>& fun,
                           const VectorBase& x,
                           const VectorBase& w,
                           VectorBase& y,
                           const VectorSet& jac_p,
                           const VectorSize& jac_row,
                           const VectorSize& jac_col,
                           VectorBase& jac,
                           const VectorSet& hes_p,
                           const VectorSize& hes_row,
                           const VectorSize& hes_col,
                           VectorBase& hes,
                           SparseForjacHessianWork& work);

template<class Base, class VectorBase, class VectorVectorBase, class VectorSet, class VectorSize>
size_t sparseForJacHessian(ADFun<Base>& fun,
                           const VectorBase& x,
                           const VectorVectorBase& w,
                           VectorBase& y,
                           const VectorSet& jac_p,
                           const VectorSize& jac_row,
                           const VectorSize& jac_col,
                           VectorBase& jac,
                           const VectorSet& hes_p,
                           const VectorSize& hes_row,
                           const VectorSize& hes_col,
                           VectorVectorBase& hes,
                           SparseForjacHessianWork& work);

/***********************************************************************
 * Sparsity evaluation
 **********************************************************************/

template<class VectorBool, class Base>
inline VectorBool jacobianForwardSparsity(ADFun<Base>& fun);

template<class VectorBool, class Base>
inline VectorBool jacobianReverseSparsity(ADFun<Base>& fun);

template<class VectorSet, class Base>
inline VectorSet jacobianForwardSparsitySet(ADFun<Base>& fun);

template<class VectorSet, class Base>
inline VectorSet jacobianReverseSparsitySet(ADFun<Base>& fun);

template<class VectorBool, class Base>
inline VectorBool jacobianSparsity(ADFun<Base>& fun);

template<class VectorSet, class Base>
inline VectorSet jacobianSparsitySet(ADFun<Base>& fun);

inline bool estimateBestJacobianADMode(const std::vector<size_t>& jacRows,
                                       const std::vector<size_t>& jacCols);

template<class VectorBool, class Base>
inline VectorBool hessianSparsity(ADFun<Base>& fun,
                                  bool transpose = false);

template<class VectorSet, class Base>
inline VectorSet hessianSparsitySet(ADFun<Base>& fun,
                                    const std::set<size_t>& w,
                                    bool transpose = false);

template<class VectorSet, class Base>
inline VectorSet hessianSparsitySet(ADFun<Base>& fun,
                                    bool transpose = false);

template<class VectorBool, class Base>
inline VectorBool hessianSparsity(ADFun<Base>& fun,
                                  size_t i,
                                  bool transpose = false);

template<class VectorSet, class Base>
inline VectorSet hessianSparsitySet(ADFun<Base>& fun,
                                    size_t i,
                                    bool transpose = false);

/***********************************************************************
 * Sparsity conversion
 **********************************************************************/

template<class VectorBool, class VectorSize>
inline void generateSparsityIndexes(const VectorBool& sparsity,
                                    size_t m,
                                    size_t n,
                                    VectorSize& row,
                                    VectorSize& col);

template<class VectorSet, class VectorSize>
inline void generateSparsityIndexes(const VectorSet& sparsity,
                                    VectorSize& row,
                                    VectorSize& col);

template<class VectorSet, class VectorSize>
inline void generateSparsitySet(const VectorSize& row,
                                const VectorSize& col,
                                VectorSet& sparsity);
}
}

#endif

