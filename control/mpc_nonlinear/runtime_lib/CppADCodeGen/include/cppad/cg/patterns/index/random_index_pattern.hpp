#ifndef CPPAD_CG_RANDOM_INDEX_PATTERN_INCLUDED
#define CPPAD_CG_RANDOM_INDEX_PATTERN_INCLUDED
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
class RandomIndexPattern : public IndexPattern {
protected:
    std::string name_;
public:

    inline void getSubIndexes(std::set<IndexPattern*>& indexes) const override {
        // nothing to add
    }

    inline const std::string& getName() const {
        return name_;
    }

    inline void setName(const std::string& name) {
        name_ = name;
    }

    inline virtual ~RandomIndexPattern() = default;
};

} // END cg namespace
} // END CppAD namespace

#endif
