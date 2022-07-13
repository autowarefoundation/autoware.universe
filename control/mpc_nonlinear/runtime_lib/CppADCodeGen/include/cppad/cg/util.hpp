#ifndef CPPAD_CG_UTIL_INCLUDED
#define CPPAD_CG_UTIL_INCLUDED
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

template<class VectorBool, class Base>
void zeroOrderDependency(ADFun<Base>& fun,
                         const VectorBool& vx,
                         VectorBool& vy) {
    size_t m = fun.Range();
    CPPADCG_ASSERT_KNOWN(vx.size() >= fun.Domain(), "Invalid vx size")
    CPPADCG_ASSERT_KNOWN(vy.size() >= m, "Invalid vy size")

    using VectorSet = std::vector<std::set<size_t> >;

    const VectorSet jacSparsity = jacobianSparsitySet<VectorSet, Base>(fun);

    for (size_t i = 0; i < m; i++) {
        for (size_t j : jacSparsity[i]) {
            if (vx[j]) {
                vy[i] = true;
                break;
            }
        }
    }
}

template<class VectorSet>
inline bool isIdentityPattern(const VectorSet& pattern,
                              size_t mRows) {
    CPPADCG_ASSERT_UNKNOWN(pattern.size() >= mRows)

    for (size_t i = 0; i < mRows; i++) {
        if (pattern[i].size() != 1 || *pattern[i].begin() != i) {
            return false;
        }
    }
    return true;
}

template<class VectorSet>
inline VectorSet transposePattern(const VectorSet& pattern,
                                  size_t mRows,
                                  size_t nCols) {
    CPPADCG_ASSERT_UNKNOWN(pattern.size() >= mRows)

    VectorSet transpose(nCols);
    for (size_t i = 0; i < mRows; i++) {
        for (size_t it : pattern[i]) {
            transpose[it].insert(i);
        }
    }
    return transpose;
}

/**
 * Computes the resulting sparsity from adding a transpose of a matrix
 * to another matrix:
 * R += A^T
 *
 * @param a The matrix to be added to the result
 * @param mRows number of rows of A to use
 * @param result the resulting sparsity matrix
 */
template<class VectorSet, class VectorSet2>
inline void addTransMatrixSparsity(const VectorSet& a,
                                   size_t mRows,
                                   VectorSet2& result) {
    CPPADCG_ASSERT_UNKNOWN(a.size() >= mRows)

    for (size_t i = 0; i < mRows; i++) {
        for (size_t j : a[i]) {
            result[j].insert(i);
        }
    }
}

/**
 * Computes the resulting sparsity from adding a transpose of a matrix
 * to another matrix:
 * R += A^T
 *
 * @param a The matrix to be added to the result
 * @param result the resulting sparsity matrix
 */
template<class VectorSet, class VectorSet2>
inline void addTransMatrixSparsity(const VectorSet& a,
                                   VectorSet2& result) {
    addTransMatrixSparsity<VectorSet, VectorSet2>(a, a.size(), result);
}

/**
 * Computes the resulting sparsity from adding one matrix to another:
 * R += A
 *
 * @param a The matrix to be added to the result
 * @param mRows number of rows of A to use
 * @param result the resulting sparsity matrix
 */
template<class VectorSet, class VectorSet2>
inline void addMatrixSparsity(const VectorSet& a,
                              size_t mRows,
                              VectorSet2& result) {
    CPPADCG_ASSERT_UNKNOWN(result.size() >= mRows)
    CPPADCG_ASSERT_UNKNOWN(a.size() <= mRows)

    for (size_t i = 0; i < mRows; i++) {
        if (result[i].empty()) {
            result[i] = a[i];
        } else {
            result[i].insert(a[i].begin(), a[i].end());
        }
    }
}

/**
 * Computes the resulting sparsity from adding one matrix to another:
 * R += A
 *
 * @param a The matrix to be added to the result
 * @param result the resulting sparsity matrix
 */
template<class VectorSet, class VectorSet2>
inline void addMatrixSparsity(const VectorSet& a,
                              VectorSet2& result) {
    CPPADCG_ASSERT_UNKNOWN(result.size() == a.size())

    addMatrixSparsity<VectorSet, VectorSet2>(a, a.size(), result);
}

/**
 * Computes the resulting sparsity from the multiplying of two matrices:
 * R += A * B
 *
 * @param a The left matrix in the multiplication
 * @param b The right matrix in the multiplication
 * @param result the resulting sparsity matrix
 * @param q The number of columns of B and the result
 */
template<class VectorSet, class VectorSet2>
inline void multMatrixMatrixSparsity(const VectorSet& a,
                                     const VectorSet2& b,
                                     CppAD::vector<std::set<size_t> >& result,
                                     size_t q) {
    multMatrixMatrixSparsity(a, b, result, a.size(), b.size(), q);
}

/**
 * Computes the resulting sparsity from the multiplying of two matrices:
 * R += A * B
 *
 * Optimized for when the B matrix has less elements than A.
 *
 * @param a The left matrix in the multiplication
 * @param b The right matrix in the multiplication
 * @param result the resulting sparsity matrix
 * @param m The number of rows of A
 * @param n The number of columns of A and rows of B
 * @param q The number of columns of B and the result
 */
template<class VectorSet, class VectorSet2>
inline void multMatrixMatrixSparsity(const VectorSet& a,
                                     const VectorSet2& b,
                                     CppAD::vector<std::set<size_t> >& result,
                                     size_t m,
                                     size_t n,
                                     size_t q) {
    CPPADCG_ASSERT_UNKNOWN(a.size() >= m)
    CPPADCG_ASSERT_UNKNOWN(b.size() >= n)
    CPPADCG_ASSERT_UNKNOWN(result.size() >= m)

    //check if b is identity
    if (n == q) {
        if (isIdentityPattern(b, n)) {
            addMatrixSparsity(a, m, result); // R += A
            return;
        }
    }

    VectorSet2 bt = transposePattern(b, n, q);

    for (size_t jj = 0; jj < q; jj++) { //loop columns of b
        const std::set<size_t>& colB = bt[jj];
        if (!colB.empty()) {
            for (size_t i = 0; i < m; i++) {
                const std::set<size_t>& rowA = a[i];
                for (size_t rowb : colB) {
                    if (rowA.find(rowb) != rowA.end()) {
                        result[i].insert(jj);
                        break;
                    }
                }
            }
        }
    }
}

/**
 * Computes the resulting sparsity from multiplying two matrices:
 * R += A^T * B
 *
 * Optimized for when the B matrix has less elements than A.
 *
 * @param a The left matrix in the multiplication
 * @param b The right matrix in the multiplication
 * @param result the resulting sparsity matrix
 * @param m The number of rows of A and rows of B
 * @param n The number of columns of A and rows of the result
 * @param q The number of columns of B and the result
 */
template<class VectorSet, class VectorSet2>
inline void multMatrixTransMatrixSparsity(const VectorSet& a,
                                          const VectorSet2& b,
                                          CppAD::vector<std::set<size_t> >& result,
                                          size_t m,
                                          size_t n,
                                          size_t q) {
    CPPADCG_ASSERT_UNKNOWN(a.size() >= m)
    CPPADCG_ASSERT_UNKNOWN(b.size() >= m)
    CPPADCG_ASSERT_UNKNOWN(result.size() >= n)

    //check if B is empty
    bool empty = true;
    for (size_t i = 0; i < m; i++) {
        if (b[i].size() > 0) {
            empty = false;
            break;
        }
    }
    if (empty) {
        return; //nothing to do: R += 0
    }

    //check if A is identity
    if (m == n && isIdentityPattern(a, m)) {
        addMatrixSparsity(b, n, result); // R += B
        return;
    }

    //check if B is identity
    if (m == q && isIdentityPattern(b, m)) {
        addTransMatrixSparsity(a, m, result); // R += A^T
        return;
    }

    VectorSet at = transposePattern(a, m, n);
    VectorSet2 bt = transposePattern(b, m, q);

    for (size_t jj = 0; jj < q; jj++) { //loop columns of b
        const std::set<size_t>& colB = bt[jj];
        if (!colB.empty()) {
            for (size_t i = 0; i < n; i++) {
                const std::set<size_t>& rowAt = at[i];
                if (!rowAt.empty()) {
                    for (size_t rowb : colB) {
                        if (rowAt.find(rowb) != rowAt.end()) {
                            result[i].insert(jj);
                            break;
                        }
                    }
                }
            }
        }
    }

}

/**
 * Computes the transpose of the resulting sparsity from multiplying two
 * matrices:
 * (R += A * B)^T
 *
 * @param a The TRANSPOSE of the left matrix in the multiplication
 * @param b The right matrix in the multiplication
 * @param result the TRANSPOSE of the resulting sparsity matrix
 * @param m The number of rows of B
 * @param n The number of columns of B
 * @param q The number of rows of A and the result
 */
template<class VectorSet, class VectorSet2>
inline void multMatrixMatrixSparsityTrans(const VectorSet& aT,
                                          const VectorSet2& b,
                                          CppAD::vector<std::set<size_t> >& rT,
                                          size_t m,
                                          size_t n,
                                          size_t q) {
    CPPADCG_ASSERT_UNKNOWN(aT.size() >= m)
    CPPADCG_ASSERT_UNKNOWN(b.size() >= m)

    //check if b is empty
    bool empty = true;
    for (size_t i = 0; i < m; i++) {
        if (b[i].size() > 0) {
            empty = false;
            break;
        }
    }
    if (empty) {
        return; //nothing to do:  R^T += 0
    }

    //check if a is identity
    if (m == q && isIdentityPattern(aT, m)) {
        addTransMatrixSparsity(b, m, rT); // R^T += B^T
        return;
    }

    VectorSet a = transposePattern(aT, m, q);
    VectorSet2 bT = transposePattern(b, m, n);

    for (size_t jj = 0; jj < n; jj++) { //loop columns of b
        for (size_t i = 0; i < q; i++) {
            for (size_t it : a[i]) {
                if (bT[jj].find(it) != bT[jj].end()) {
                    rT[jj].insert(i);
                    break;
                }
            }
        }
    }
}

template<class VectorBool>
void printSparsityPattern(const VectorBool& sparsity,
                          const std::string& name,
                          size_t m, size_t n) {
    size_t width = std::ceil(std::log10((m > n) ? m : n));
    if (!name.empty()) {
        std::cout << name << "  sparsity:\n";
    }
    for (size_t i = 0; i < m; i++) {
        std::cout << " " << std::setw(width) << i << ": ";
        for (size_t j = 0; j < n; j++) {
            if (sparsity[i * n + j]) {
                std::cout << std::setw(width) << j << " ";
            } else {
                std::cout << std::setw(width) << " " << " ";
            }
        }
        std::cout << "\n";
    }
    std::cout << std::endl;
}

template<class VectorSet>
void printSparsityPattern(const VectorSet& sparsity,
                          const std::string& name,
                          bool printLocationByRow = false) {
    size_t maxDim = sparsity.size();
    size_t nnz = 0;
    for (size_t i = 0; i < sparsity.size(); i++) {
        if (sparsity[i].size() > 0 && *sparsity[i].rbegin() > maxDim) {
            maxDim = *sparsity[i].rbegin();
        }
        nnz += sparsity[i].size();
    }

    size_t width = std::ceil(std::log10(maxDim));
    size_t width2;
    size_t width3 = width;
    if (printLocationByRow) {
        width2 = std::ceil(std::log10(nnz));
        width3 += width2 + 1;
    }
    if (!name.empty()) {
        std::cout << name << "  sparsity:\n";
    }

    size_t e = 0;
    for (size_t i = 0; i < sparsity.size(); i++) {
        std::cout << " " << std::setw(width) << i << ": ";
        long last = -1;
        for (size_t j : sparsity[i]) {
            if (j != 0 && long(j) != last + 1) {
                std::cout << std::setw((j - last - 1) * (width3 + 1)) << " ";
            }
            if (printLocationByRow)
                std::cout << std::setw(width2) << e << ":";
            std::cout << std::setw(width) << j << " ";
            last = j;
            e++;
        }
        std::cout << "\n";
    }
    std::cout << std::endl;
}

template<class VectorSize>
void printSparsityPattern(const VectorSize& row,
                          const VectorSize& col,
                          const std::string& name,
                          size_t m) {
    std::vector<std::set<size_t> > sparsity(m);
    generateSparsitySet(row, col, sparsity);
    printSparsityPattern(sparsity, name);
}

inline bool intersects(const std::set<size_t>& a,
                       const std::set<size_t>& b) {
    if (a.empty() || b.empty()) {
        return false;
    } else if (*a.rbegin() < *b.begin() ||
            *a.begin() > *b.rbegin()) {
        return false;
    }

    if (a.size() < b.size()) {
        for (size_t ita : a) {
            if (b.find(ita) != b.end()) {
                return true;
            }
        }
    } else {
        for (size_t itb : b) {
            if (a.find(itb) != a.end()) {
                return true;
            }
        }
    }

    return false;
}

template<class VectorSizet, class VectorSet>
inline CppAD::sparse_rc<VectorSizet> toSparsityPattern(const VectorSet& inPattern,
                                                       size_t m, size_t n) {

    CppAD::sparse_rc<VectorSizet> pattern;

    size_t nnz = 0;
    for (const auto& p: inPattern) {
        nnz += p.size();
    }

    pattern.resize(m, n, nnz);

    size_t e = 0;
    for (size_t i = 0; i < inPattern.size(); ++i) {
        for (size_t j : inPattern[i]) {
            pattern.set(e++, i, j);
        }
    }

    return pattern;
}

/**
 * Finds the first non-null code handler
 *
 * @param ty The array to search in
 * @return The first code handler found or nullptr if none was found
 */
template<class Base>
inline CodeHandler<Base>* findHandler(const std::vector<CG<Base> >& ty) {
    for (size_t i = 0; i < ty.size(); i++) {
        if (ty[i].getCodeHandler() != nullptr) {
            return ty[i].getCodeHandler();
        }
    }
    return nullptr;
}

template<class Base>
inline CodeHandler<Base>* findHandler(const CppAD::vector<CG<Base> >& ty) {
    for (size_t i = 0; i < ty.size(); i++) {
        if (ty[i].getCodeHandler() != nullptr) {
            return ty[i].getCodeHandler();
        }
    }
    return nullptr;
}

template<class Base>
inline CodeHandler<Base>* findHandler(CppAD::cg::ArrayView<const CG<Base> > ty) {
    for (size_t i = 0; i < ty.size(); i++) {
        if (ty[i].getCodeHandler() != nullptr) {
            return ty[i].getCodeHandler();
        }
    }
    return nullptr;
}

template<class Base>
inline Argument<Base> asArgument(const CG<Base>& tx) {
    if (tx.isParameter()) {
        return Argument<Base>(tx.getValue());
    } else {
        return Argument<Base>(*tx.getOperationNode());
    }
}

template<class Base>
inline std::vector<Argument<Base> > asArguments(const std::vector<CG<Base> >& tx) {
    std::vector<Argument<Base> > arguments(tx.size());
    for (size_t i = 0; i < arguments.size(); i++) {
        arguments[i] = asArgument(tx[i]);
    }
    return arguments;
}

template<class Base>
inline std::vector<Argument<Base> > asArguments(const CppAD::vector<CG<Base> >& tx) {
    std::vector<Argument<Base> > arguments(tx.size());
    for (size_t i = 0; i < arguments.size(); i++) {
        arguments[i] = asArgument(tx[i]);
    }
    return arguments;
}

/***************************************************************************
 * map related
 **************************************************************************/

/**
 * Gets all the keys present in a map
 *
 * @param map the map from which to get the keys from
 * @param keys the map keys will be inserted into this set
 */
template<class Key, class Value>
void mapKeys(const std::map<Key, Value>& map, std::set<Key>& keys) {
    for (const auto& p : map) {
        keys.insert(keys.end(), p.first);
    }
}

/**
 * Gets all the keys present in a map
 *
 * @param map the map from which to get the keys from
 * @param keys the map keys will be saved in this vector
 */
template<class Key, class Value>
void mapKeys(const std::map<Key, Value>& map, std::vector<Key>& keys) {
    keys.resize(map.size());

    size_t i = 0;
    typename std::map<Key, Value>::const_iterator it;
    for (it = map.begin(); it != map.end(); ++it, i++) {
        keys[i] = it->first;
    }
}

/**
 * Checks if a map has only a set of keys.
 *
 * @param map The map
 * @param keys The keys
 * @return true if all the keys and only these keys where found in the map
 */
template<class Key, class Value>
bool compareMapKeys(const std::map<Key, Value>& map, const std::set<Key>& keys) {
    if (map.size() != keys.size())
        return false;

    typename std::map<Key, Value>::const_iterator itm = map.begin();
    typename std::set<Key>::const_iterator itk = keys.begin();
    for (; itm != map.end(); ++itm, ++itk) {
        if (itm->first != *itk)
            return false;
    }

    return true;
}

/**
 * Creates a new map with only a given set of keys
 *
 * @param m The map to be filtered
 * @param keys the keys (the filter) to be retrieved from the map
 * @return a new map only with the keys found in provided filter
 */
template<class Key, class Value>
inline std::map<Key, Value> filterBykeys(const std::map<Key, Value>& m,
                                         const std::set<Key>& keys) {
    std::map<Key, Value> filtered;

    typename std::map<Key, Value>::const_iterator itM;

    for (const Key& k : keys) {
        itM = m.find(k);
        if (itM != m.end()) {
            filtered[itM->first] = itM->second;
        }
    }
    return filtered;
}

/**
 * Compares two sets
 *
 * @param s1 the first set
 * @param s2 the second set
 * @return -1 if the first set is considered lower than the second,
 *         0 if they have all the same elements
 *         1 if the second set is considered lower than the first.
 */
template<class T>
inline int compare(const std::set<T>& s1, const std::set<T>& s2) {
    if (s1.size() < s2.size()) {
        return -1;
    } else if (s1.size() > s2.size()) {
        return 1;
    } else {
        typename std::set<T>::const_iterator it1, it2;
        for (it1 = s1.begin(), it2 = s2.begin(); it1 != s1.end(); ++it1, ++it2) {
            if (*it1 < *it2) {
                return -1;
            } else if (*it1 > *it2) {
                return 1;
            }
        }
        return 0;
    }
}

template<class T>
struct SetComparator {

    bool operator() (const std::set<T>& lhs, const std::set<T>& rhs) const {
        return compare(lhs, rhs) == -1;
    }
};

/***************************************************************************
 * Generic functions for printing stl containers
 **************************************************************************/
template<class Base>
inline void print(const Base& v) {
    std::cout << v;
}

template<class Key, class Value>
inline void print(const std::map<Key, Value>& m) {
    for (const std::pair<Key, Value>& p : m) {
        std::cout << p.first << " : ";
        print(p.second);
        std::cout << std::endl;
    }
}

template<class Base>
inline void print(const std::set<Base>& s) {
    std::cout << "[";

    for (auto itj = s.begin(); itj != s.end(); ++itj) {
        if (itj != s.begin()) std::cout << " ";
        print(*itj);
    }
    std::cout << "]";
    std::cout.flush();
}

template<class Base>
inline void print(const std::set<Base*>& s) {
    std::cout << "[";

    for (const auto itj = s.begin(); itj != s.end(); ++itj) {
        if (itj != s.begin()) std::cout << " ";
        Base* v = *itj;
        if (v == nullptr) std::cout << "NULL";
        else print(*v);
    }
    std::cout << "]";
    std::cout.flush();
}

template<class Base>
inline void print(const std::vector<Base>& v) {
    std::cout << "[";

    for (size_t i = 0; i < v.size(); i++) {
        if (i != 0) std::cout << " ";
        print(v[i]);
    }
    std::cout << "]";
    std::cout.flush();
}

template<class VectorSize, class VectorBase>
inline void printTripletMatrix(const VectorSize &rows,
                               const VectorSize &cols,
                               const VectorBase &values) {
    size_t n = values.size();
    assert(rows.size() == n);
    assert(cols.size() == n);

    for (size_t i = 0; i < n; ++i) {
        std::cout << "[ " << rows[i] << ", " << cols[i] << "] -> " << values[i] << std::endl;
    }
}

/***************************************************************************
 * print
 **************************************************************************/
/**
 * It prints the provided value to standard output.
 * If the provided input is a variable, that is, it has an OperationNode,
 * it also creates a CG<value> with a print operation which is returned.
 *
 * @param before a string to be printed before the value
 * @param x the input that will be printed
 * @param after a string to be printed after the value.
 * @return a value with the print operation if x is a variable, otherwise a
 *         copy of x
 */
template<class Base>
inline CG<Base> makePrintValue(const std::string& before,
                               const CG<Base>& x,
                               const std::string& after = "") {
    std::cout << before << x << after;

    if (x.getOperationNode() != nullptr) {
        auto* handler = x.getCodeHandler();
        CG<Base> out(*handler->makePrintNode(before, *x.getOperationNode(), after));
        if (x.isValueDefined())
            out.setValue(x.getValue());
        return out;
    } else {
        return x;
    }
}

/***************************************************************************
 * String related utilities
 **************************************************************************/

/**
 * Replaces all occurrence of a string.
 *
 * @param text the text where the search and replacement will be performed
 * @param toReplace the text to be replaced
 * @param replacement the replacement text
 */
inline void replaceString(std::string& text,
                          const std::string& toReplace,
                          const std::string& replacement) {
    size_t pos = 0;
    while ((pos = text.find(toReplace, pos)) != std::string::npos) {
        text.replace(pos, toReplace.length(), replacement);
        pos += replacement.length();
    }
}

inline std::vector<std::string> explode(const std::string& text,
                                        const std::string& delimiter) {
    std::vector<std::string> matches;

    const size_t dlen = delimiter.length();
    if (dlen == 0)
        return matches;

    size_t pos = 0;
    size_t start = 0;
    while (true) {
        pos = text.find(delimiter, start);
        if (pos == std::string::npos) {
            break;
        }
        matches.push_back(text.substr(start, pos - start));
        start = pos + dlen;
    }

    if (start < text.length()) {
        matches.push_back(text.substr(start, text.length() - start));
    }

    return matches;
}

inline std::string implode(const std::vector<std::string>& text,
                           const std::string& delimiter) {
    if (text.empty()) {
        return "";
    } else if (text.size() == 1) {
        return text[0];
    } else {
        std::string out;
        size_t n = 0;
        for (const auto& s: text)
            n += s.size();
        out.reserve(n + (text.size() - 1) * delimiter.size());
        out = text[0];
        for (size_t i = 1; i < text.size(); ++i) {
            out += delimiter;
            out += text[i];
        }
        return out;
    }
}

inline std::string readStringFromFile(const std::string& path) {
    std::ifstream iStream;
    iStream.open(path);

    std::stringstream strStream;
    strStream << iStream.rdbuf();

    return strStream.str();
}

} // END cg namespace
} // END CppAD namespace

#endif
