#ifndef CPPAD_CG_SECTIONED_INDEX_PATTERN_INCLUDED
#define CPPAD_CG_SECTIONED_INDEX_PATTERN_INCLUDED
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
 * Several linear patterns
 */
class SectionedIndexPattern : public IndexPattern {
protected:
    /**
     * maps the start of the linear section (first x) to the linear pattern
     */
    std::map<size_t, IndexPattern*> sections_;
public:

    inline SectionedIndexPattern(const std::map<size_t, IndexPattern*>& sections) :
        sections_(sections) {
    }

    inline const std::map<size_t, IndexPattern*>& getLinearSections() const {
        return sections_;
    }

    inline IndexPatternType getType() const override {
        return IndexPatternType::Sectioned;
    }

    inline void getSubIndexes(std::set<IndexPattern*>& indexes) const override {
        for (const auto& it : sections_) {
            indexes.insert(it.second);
            it.second->getSubIndexes(indexes);
        }
    }

    inline virtual ~SectionedIndexPattern() {
        deleteIndexPatterns(sections_);
    }

    /***********************************************************************
     *                        static methods
     **********************************************************************/

    template<class VectorSizeT>
    static inline std::map<size_t, IndexPattern*> detectLinearSections(const VectorSizeT& indexes,
                                                                       size_t maxCount = 0) {
        CPPADCG_ASSERT_UNKNOWN(indexes.size() > 0);

        long dx = 1;
        long xOffset = 0;

        LinearIndexPattern* prevPattern = nullptr;
        size_t prevXStart = 0;

        SmartMapValuePointer<size_t, IndexPattern> linearSections;
        size_t xStart = 0;
        while (xStart != indexes.size()) {
            long dy, b;
            size_t lastLinear;
            if (xStart + 1 == indexes.size()) {
                dy = 0;
                b = indexes[xStart];
                lastLinear = xStart + 1;
            } else {
                dy = long(indexes[xStart + 1]) - indexes[xStart];
                b = long(indexes[xStart]) - dy * xStart;
                lastLinear = indexes.size();
                for (size_t x = xStart + 2; x < indexes.size(); x++) {
                    if (indexes[x] != dy * x + b) {
                        lastLinear = x;
                        break;
                    }
                }
            }

            LinearIndexPattern* p = new LinearIndexPattern(xOffset, dy, dx, b);
            if (dy == 0 && prevPattern != nullptr) { // constant
                // can we take the last element out of the previous section?
                while (xStart > 0 && prevPattern->evaluate(xStart - 1) == b) {
                    // yes
                    xStart--;
                    if (prevXStart + 1 == xStart) {
                        // it has only one element -> make it a constant section
                        size_t bb = prevPattern->evaluate(prevXStart);
                        prevPattern->setLinearSlopeDy(0);
                        prevPattern->setLinearConstantTerm(bb);
                    }
                }
            }
            linearSections[xStart] = p;

            prevXStart = xStart;
            prevPattern = p;
            xStart = lastLinear;

            if (linearSections.size() == maxCount && xStart != indexes.size()) {
                // over the limit -> stop
                return std::map<size_t, IndexPattern*>(); // empty
            }
        }

        return linearSections.release();
    }

    static inline std::map<size_t, IndexPattern*> detectLinearSections(const std::map<size_t, size_t>& x2y,
                                                                       size_t maxCount = 0) {
        using c_iter = std::map<size_t, size_t>::const_iterator;

        SmartMapValuePointer<size_t, IndexPattern> linearSections;

        LinearIndexPattern* prevPattern = nullptr;
        c_iter prevStart = x2y.begin();

        c_iter pStart = x2y.begin();
        while (pStart != x2y.end()) {
            c_iter pNextSection = x2y.end();
            c_iter p1 = pStart;
            ++p1;

            long xOffset, dy, dx, b;
            if (p1 == x2y.end()) {
                xOffset = 0;
                dy = 0;
                dx = 1;
                b = pStart->second;
                pNextSection = p1;

            } else {
                long x0 = pStart->first;
                long y0 = pStart->second;

                dy = long(p1->second) - y0;
                if (dy != 0) {
                    dx = long(p1->first) - x0;
                    xOffset = x0 % dx;
                    // y = ((x - offset) / dx) * dy + b
                    b = y0 - ((x0 - xOffset) / dx) * dy;
                } else {
                    dx = 1;
                    xOffset = 0;
                    b = y0;
                }

                for (std::map<size_t, size_t>::const_iterator itp = p1; itp != x2y.end(); ++itp) {
                    size_t x = itp->first;
                    size_t y = itp->second;

                    if (y != ((x - xOffset) / dx) * dy + b) {
                        pNextSection = itp;
                        break;
                    }
                }
            }

            LinearIndexPattern* p = new LinearIndexPattern(xOffset, dy, dx, b);
            size_t xStart = pStart->first;
            if (dy == 0 && prevPattern != nullptr) { // constant
                // can we take the last element from the previous section?

                while (pStart != x2y.begin()) {
                    c_iter prevBack = pStart;
                    --prevBack; // the values at the end of the previous section
                    if (prevPattern->evaluate(prevBack->first) != b)
                        break; // no

                    // yes
                    --pStart;
                    xStart = pStart->first;
                    c_iter prevStartN = prevStart;
                    prevStartN++;
                    if (prevStartN == pStart) {
                        // it has only one element -> make it a constant section
                        size_t bb = prevPattern->evaluate(prevStart->first);
                        prevPattern->setLinearSlopeDy(0);
                        prevPattern->setLinearConstantTerm(bb);
                    }
                }
            }
            linearSections[xStart] = p;

            prevStart = pStart;
            prevPattern = p;
            pStart = pNextSection;

            if (linearSections.size() == maxCount && pStart != x2y.end()) {
                // over the limit -> stop
                return std::map<size_t, IndexPattern*>(); // empty
            }
        }

        return linearSections.release();
    }

private:

    static inline void deleteIndexPatterns(std::map<size_t, IndexPattern*>& sections) {
        for (const auto& it : sections) {
            delete it.second;
        }
        sections.clear();
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
