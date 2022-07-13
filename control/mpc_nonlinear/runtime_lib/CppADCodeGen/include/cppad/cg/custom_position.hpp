#ifndef CPPAD_CG_CUSTOM_POSITION_INCLUDED
#define CPPAD_CG_CUSTOM_POSITION_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

/**
 * Useful class for storing matrix indexes
 */
class CustomPosition {
private:
    bool filterDefined_;
    /// allowed elements
    std::vector<std::vector<bool> > elFilter_;
    bool fullDefined_;
    std::vector<std::set<size_t> > elements_;
public:

    inline CustomPosition() :
        filterDefined_(false),
        fullDefined_(false) {
    }

    template<class VectorSize>
    inline CustomPosition(size_t m, size_t n,
                          const VectorSize& rows,
                          const VectorSize& cols) :
        filterDefined_(true),
        elFilter_(m, std::vector<bool>(n, false)),
        fullDefined_(false) {
        CPPADCG_ASSERT_KNOWN(rows.size() == cols.size(), "The number of row indexes must be the same as the number of column indexes.")
        for (size_t i = 0; i < rows.size(); i++) {
            elFilter_[rows[i]][cols[i]] = true;
        }
    }

    template<class VectorSet>
    inline CustomPosition(size_t m, size_t n,
                          const VectorSet& elements) :
        filterDefined_(true),
        elFilter_(m, std::vector<bool>(n, false)),
        fullDefined_(false) {
        CPPADCG_ASSERT_KNOWN(elements.size() <= m, "Invalid number of rows.")

        for (size_t i = 0; i < elements.size(); i++) {
            for (size_t it : elements[i]) {
                elFilter_[i][it] = true;
            }
        }
    }

    inline bool isFilterDefined() const {
        return filterDefined_;
    }

    inline bool isFullDefined() const {
        return fullDefined_;
    }

    inline void setFullElements(const std::vector<std::set<size_t> >& elements) {
        elements_ = elements;
        filter(elements_);
        fullDefined_ = true;
    }

    inline const std::vector<std::set<size_t> >& getFullElements()const {
        return elements_;
    }

    inline void filter(CppAD::vector<std::set<size_t> >& sparsity) const {
        ArrayView<std::set<size_t> > s(sparsity);
        filter(s);
    }

    inline void filter(std::vector<std::set<size_t> >& sparsity) const {
        ArrayView<std::set<size_t> > s(sparsity);
        filter(s);
    }

    inline void filter(ArrayView<std::set<size_t> >& sparsity) const {
        if (!filterDefined_)
            return; // nothing to do

        std::set<size_t>::iterator it, currentIt;

        for (size_t i = 0; i < sparsity.size(); i++) {
            it = sparsity[i].begin();
            while (it != sparsity[i].end()) {
                // copy the current iterator then increment it
                currentIt = it++;
                if (!elFilter_[i][*currentIt]) {
                    sparsity[i].erase(currentIt); // not in allowed elements
                }
            }
        }
    }

};

} // END cg namespace
} // END CppAD namespace

#endif