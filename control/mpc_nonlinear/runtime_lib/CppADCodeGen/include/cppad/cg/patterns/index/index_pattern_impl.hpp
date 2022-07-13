#ifndef CPPAD_CG_INDEX_PATTERN_IMPL_INCLUDED
#define CPPAD_CG_INDEX_PATTERN_IMPL_INCLUDED
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

template<class VectorSizeT>
IndexPattern* IndexPattern::detect(const VectorSizeT& x2y) {
    CPPADCG_ASSERT_UNKNOWN(x2y.size() > 0)

    size_t maxCount = std::min<size_t>(std::max<size_t>(3ul, x2y.size() / 4), 8ul);
    std::map<size_t, IndexPattern*> linearSections = SectionedIndexPattern::detectLinearSections(x2y, maxCount);

    if (linearSections.size() == 1) {
        return linearSections.begin()->second;
    } else if (!linearSections.empty()) {
        return new SectionedIndexPattern(linearSections);
    } else {
        return new Random1DIndexPattern(x2y);
    }

}

IndexPattern* IndexPattern::detect(const std::map<size_t, size_t>& x2y) {
    CPPADCG_ASSERT_UNKNOWN(!x2y.empty())

    size_t maxCount = std::min<size_t>(std::max<size_t>(3ul, x2y.size() / 4), 8ul);
    std::map<size_t, IndexPattern*> linearSections = SectionedIndexPattern::detectLinearSections(x2y, maxCount);

    if (linearSections.size() == 1) {
        return linearSections.begin()->second;
    } else if (!linearSections.empty()) {
        return new SectionedIndexPattern(linearSections);
    } else {
        return new Random1DIndexPattern(x2y);
    }
}

inline bool IndexPattern::isConstant(const IndexPattern& ip) {
    if (ip.getType() == IndexPatternType::Linear) {
        const auto& lip = static_cast<const LinearIndexPattern&> (ip);
        return lip.getLinearSlopeDy() == 0;
    }
    return false;
}

} // END cg namespace
} // END CppAD namespace

#endif