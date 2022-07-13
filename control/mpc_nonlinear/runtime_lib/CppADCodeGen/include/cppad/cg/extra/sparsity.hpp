#ifndef CPPAD_CG_SPARSITY_INCLUDED
#define CPPAD_CG_SPARSITY_INCLUDED
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

namespace CppAD {
namespace cg {

template<class VectorBool, class Base>
inline VectorBool jacobianForwardSparsity(ADFun<Base>& fun) {
    size_t n = fun.Domain();

    VectorBool r(n * n);
    for (size_t j = 0; j < n; j++) {
        for (size_t k = 0; k < n; k++)
            r[j * n + k] = false;
        r[j * n + j] = true;
    }
    return fun.ForSparseJac(n, r);

}

template<class VectorBool, class Base>
inline VectorBool jacobianReverseSparsity(ADFun<Base>& fun) {
    size_t m = fun.Range();

    VectorBool s(m * m);
    for (size_t i = 0; i < m; i++) {
        for (size_t k = 0; k < m; k++)
            s[i * m + k] = false;
        s[i * m + i] = true;
    }
    return fun.RevSparseJac(m, s);
}

template<class VectorSet, class Base>
inline VectorSet jacobianForwardSparsitySet(ADFun<Base>& fun) {
    size_t n = fun.Domain();

    VectorSet r(n);
    for (size_t i = 0; i < n; i++)
        r[i].insert(i);

    return fun.ForSparseJac(n, r);
}

template<class VectorSet, class Base>
inline VectorSet jacobianReverseSparsitySet(ADFun<Base>& fun) {
    size_t m = fun.Range();

    VectorSet s_s(m);
    for (size_t i = 0; i < m; i++)
        s_s[i].insert(i);

    return fun.RevSparseJac(m, s_s);
}

/**
 * Determines the Jacobian sparsity for a model
 * 
 * @param fun The model
 * @return The Jacobian sparsity
 */
template<class VectorBool, class Base>
inline VectorBool jacobianSparsity(ADFun<Base>& fun) {
    size_t m = fun.Range();
    size_t n = fun.Domain();

    if (n <= m) {
        // use forward mode 
        return jacobianForwardSparsity<VectorBool, Base> (fun);
    } else {
        // use reverse mode 
        return jacobianReverseSparsity<VectorBool, Base> (fun);
    }
}

/**
 * Determines the Jacobian sparsity for a model
 * 
 * @param fun The model
 * @return The Jacobian sparsity
 */
template<class VectorSet, class Base>
inline VectorSet jacobianSparsitySet(ADFun<Base>& fun) {
    size_t m = fun.Range();
    size_t n = fun.Domain();

    if (n <= m) {
        // use forward mode 
        return jacobianForwardSparsitySet<VectorSet, Base> (fun);
    } else {
        // use reverse mode 
        return jacobianReverseSparsitySet<VectorSet, Base> (fun);
    }
}

/**
 * Estimates the work load of forward vs reverse mode for the evaluation of
 * a Jacobian
 * 
 * @return true if the foward mode should be used, false for the reverse mode
 */
inline bool estimateBestJacobianADMode(const std::vector<size_t>& jacRows,
                                       const std::vector<size_t>& jacCols) {
    std::set<size_t> rows, cols;
    rows.insert(jacRows.begin(), jacRows.end());
    size_t workReverse = rows.size();
    cols.insert(jacCols.begin(), jacCols.end());
    size_t workForward = cols.size();

    return workForward <= workReverse;
}

/**
 * Determines the sum of the hessian sparsities for all the dependent 
 * variables in a model
 * 
 * @param fun The model
 * @return The sum of the hessian sparsities
 */
template<class VectorBool, class Base>
inline VectorBool hessianSparsity(ADFun<Base>& fun,
                                  bool transpose = false) {
    size_t m = fun.Range();
    size_t n = fun.Domain();

    /**
     * Determine the sparsity pattern p for Hessian of w^T F
     */
    VectorBool r(n * n); // identity matrix
    for (size_t j = 0; j < n; j++) {
        for (size_t k = 0; k < n; k++)
            r[j * n + k] = false;
        r[j * n + j] = true;
    }
    fun.ForSparseJac(n, r);

    VectorBool s(m);
    for (size_t i = 0; i < m; i++)
        s[i] = true;
    return fun.RevSparseHes(n, s, transpose);
}

template<class VectorSet, class Base>
inline VectorSet hessianSparsitySet(ADFun<Base>& fun,
                                    const std::set<size_t>& w,
                                    bool transpose = false) {
    size_t n = fun.Domain();

    /**
     * Determine the sparsity pattern p for Hessian of w^T F
     */
    VectorSet r(n); // identity matrix
    for (size_t j = 0; j < n; j++)
        r[j].insert(j);
    fun.ForSparseJac(n, r);

    VectorSet s(1);
    s[0] = w;

    return fun.RevSparseHes(n, s, transpose);
}

template<class VectorSet, class Base>
inline VectorSet hessianSparsitySet(ADFun<Base>& fun, bool transpose = false) {
    size_t m = fun.Range();

    std::set<size_t> w;
    for (size_t i = 0; i < m; i++) {
        w.insert(i);
    }
    return hessianSparsitySet<VectorSet, Base>(fun, w, transpose);
}

/**
 * Determines the hessian sparsity for a given dependent variable/equation
 * in a model
 * 
 * @param fun The model
 * @param i The dependent variable/equation index
 * @return The hessian sparsity
 */
template<class VectorBool, class Base>
inline VectorBool hessianSparsity(ADFun<Base>& fun,
                                  size_t i,
                                  bool transpose = false) {
    size_t m = fun.Range();
    size_t n = fun.Domain();

    /**
     * Determine the sparsity pattern p for Hessian of w^T F
     */
    VectorBool r(n * n); // identity matrix
    for (size_t j = 0; j < n; j++) {
        for (size_t k = 0; k < n; k++)
            r[j * n + k] = false;
        r[j * n + j] = true;
    }
    fun.ForSparseJac(n, r);

    VectorBool s(m);
    for (size_t ii = 0; ii < m; ii++)
        s[ii] = false;
    s[i] = true;
    return fun.RevSparseHes(n, s, transpose);
}

template<class VectorSet, class Base>
inline VectorSet hessianSparsitySet(ADFun<Base>& fun,
                                    size_t i,
                                    bool transpose = false) {
    size_t n = fun.Domain();

    VectorSet r(n); // identity matrix
    for (size_t j = 0; j < n; j++)
        r[j].insert(j);
    fun.ForSparseJac(n, r);

    VectorSet s(1);
    s[0].insert(i);

    return fun.RevSparseHes(n, s, transpose);
}

template<class VectorBool, class VectorSize>
inline void generateSparsityIndexes(const VectorBool& sparsity,
                                    size_t m,
                                    size_t n,
                                    VectorSize& row,
                                    VectorSize& col) {
    assert(sparsity.size() == m * n);

    // determine total number of non zeros
    size_t nnz = 0;
    for (size_t i = 0; i < sparsity.size(); i++) {
        if (sparsity[i])
            nnz++;
    }

    row.resize(nnz);
    col.resize(nnz);

    // save the indexes
    nnz = 0;
    for (size_t i = 0; i < m; i++) {
        for (size_t j = 0; j < n; j++) {
            if (sparsity[i * n + j]) {
                row[nnz] = i;
                col[nnz] = j;
                nnz++;
            }
        }
    }

    assert(nnz == row.size());
}

template<class VectorSet, class VectorSize>
inline void generateSparsityIndexes(const VectorSet& sparsity,
                                    VectorSize& row,
                                    VectorSize& col) {
    size_t m = sparsity.size();

    // determine total number of non zeros
    size_t nnz = 0;
    for (size_t i = 0; i < m; i++) {
        nnz += sparsity[i].size();
    }

    row.resize(nnz);
    col.resize(nnz);
    if (nnz == 0)
        return;

    // save the indexes
    nnz = 0;
    for (size_t i = 0; i < m; i++) {
        const std::set<size_t>& rowSparsity = sparsity[i];
        size_t rowNnz = rowSparsity.size();
        std::fill(&row[0] + nnz, &row[0] + nnz + rowNnz, i);
        std::copy(rowSparsity.begin(), rowSparsity.end(), &col[0] + nnz);
        nnz += rowNnz;
    }
}

template<class VectorSet, class VectorSize>
inline void generateSparsitySet(const VectorSize& row,
                                const VectorSize& col,
                                VectorSet& sparsity) {
    assert(row.size() == col.size());

    size_t nnz = row.size();
    for (size_t e = 0; e < nnz; e++) {
        sparsity[row[e]].insert(col[e]);
    }
}

} // END cg namespace
} // END CppAD namespace

#endif

