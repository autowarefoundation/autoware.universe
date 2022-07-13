#ifndef CPPAD_CG_RANDOM_2D_INDEX_PATTERN_INCLUDED
#define CPPAD_CG_RANDOM_2D_INDEX_PATTERN_INCLUDED
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
 * Random pattern using two indexes
 */
class Random2DIndexPattern : public RandomIndexPattern {
protected:
    std::map<size_t, std::map<size_t, size_t> > indexes_;
    std::string name_;
public:

    inline Random2DIndexPattern(const std::map<size_t, std::map<size_t, size_t> >& x2y2z) :
        indexes_(x2y2z) {
        CPPADCG_ASSERT_UNKNOWN(!indexes_.empty());
    }

    inline virtual ~Random2DIndexPattern() = default;

    inline IndexPatternType getType() const override {
        return IndexPatternType::Random2D;
    }

    inline const std::map<size_t, std::map<size_t, size_t> >& getValues() const {
        return indexes_;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
