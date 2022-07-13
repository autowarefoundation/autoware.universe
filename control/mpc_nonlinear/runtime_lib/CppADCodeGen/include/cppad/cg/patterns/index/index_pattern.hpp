#ifndef CPPAD_CG_INDEX_PATTERN_INCLUDED
#define CPPAD_CG_INDEX_PATTERN_INCLUDED
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
 * Generic index pattern
 */
class IndexPattern {
public:

    virtual IndexPatternType getType() const = 0;

    virtual void getSubIndexes(std::set<IndexPattern*>& indexes) const = 0;

    inline virtual ~IndexPattern() {
    }

    /***********************************************************************
     *   static methods
     **********************************************************************/
    /**
     * Detects the index pattern for the provided points (y = f(x))
     * 
     * @param indexX the index of the independents (x)
     * @param x2y maps the independents to the dependents (indexes[x] = y )
     * @return the generated index pattern (must be deleted by user)
     */
    template<class VectorSizeT>
    static inline IndexPattern* detect(const VectorSizeT& x2y);

    /**
     * Detects the index pattern for the provided points (y = f(x))
     * 
     * @param indexX the index of the independents (x)
     * @param x2y maps the independents to the dependents (x,y)
     * @return the generated index pattern (must be deleted by user)
     */
    static inline IndexPattern* detect(const std::map<size_t, size_t>& x2y);

    static inline bool isConstant(const IndexPattern& ip);
};

} // END cg namespace
} // END CppAD namespace

#endif