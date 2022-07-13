#ifndef CPPAD_CG_LINEAR_INDEX_PATTERN_INCLUDED
#define CPPAD_CG_LINEAR_INDEX_PATTERN_INCLUDED
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
 * Linear pattern y = ((x - offset) / dx) * dy + b
 */
class LinearIndexPattern : public IndexPattern {
protected:
    long xOffset_;
    // slope
    long dy_;
    long dx_;
    // constant term
    long b_;
public:

    inline LinearIndexPattern(long xOffset, long dy, long dx, long b) :
        xOffset_(xOffset),
        dy_(dy),
        dx_(dx),
        b_(b) {
    }

    inline virtual ~LinearIndexPattern() = default;

    inline long getXOffset()const {
        return xOffset_;
    }

    inline long getLinearSlopeDy() const {
        return dy_;
    }

    inline void setLinearSlopeDy(long dy) {
        dy_ = dy;
    }

    inline long getLinearSlopeDx() const {
        return dx_;
    }

    inline long getLinearConstantTerm() const {
        return b_;
    }

    inline void setLinearConstantTerm(long b) {
        b_ = b;
    }

    inline IndexPatternType getType() const override {
        return IndexPatternType::Linear;
    }

    inline void getSubIndexes(std::set<IndexPattern*>& indexes) const override {
        // nothing to add
    }

    inline long evaluate(long x) const {
        return ((x - xOffset_) / dx_) * dy_ + b_;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
