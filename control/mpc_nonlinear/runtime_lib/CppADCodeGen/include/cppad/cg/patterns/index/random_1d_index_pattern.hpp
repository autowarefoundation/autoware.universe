#ifndef CPPAD_CG_RANDOM_1D_INDEX_PATTERN_INCLUDED
#define CPPAD_CG_RANDOM_1D_INDEX_PATTERN_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *    Copyright (C) 2018 Joao Leal
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
 * Random pattern
 */
class Random1DIndexPattern : public RandomIndexPattern {
protected:
    std::map<size_t, size_t> indexes_;
    std::string name_;
public:

    template<class VectorSizeT>
    inline Random1DIndexPattern(const VectorSizeT& x2y) {
        CPPADCG_ASSERT_UNKNOWN(x2y.size() > 0);
        for (size_t x = 0; x < x2y.size(); x++)
            indexes_[x] = x2y[x];
    }

    inline virtual ~Random1DIndexPattern() = default;

    inline Random1DIndexPattern(const std::map<size_t, size_t>& x2y) :
        indexes_(x2y) {
        CPPADCG_ASSERT_UNKNOWN(!indexes_.empty());
    }

    inline IndexPatternType getType() const override {
        return IndexPatternType::Random1D;
    }

    inline const std::map<size_t, size_t>& getValues() const {
        return indexes_;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
