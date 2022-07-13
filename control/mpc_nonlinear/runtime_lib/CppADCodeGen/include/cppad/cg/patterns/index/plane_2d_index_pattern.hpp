#ifndef CPPAD_CG_PLANE_2D_INDEX_PATTERN_INCLUDED
#define CPPAD_CG_PLANE_2D_INDEX_PATTERN_INCLUDED
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
 * A plane index pattern (2D) defined by two index patterns for each index
 * coordinate
 *
 * z = f1(x) + f2(y)
 */
class Plane2DIndexPattern : public IndexPattern {
protected:
    /**
     * maps the start of the linear section (first x) to the linear pattern
     */
    IndexPattern* pattern1_;
    IndexPattern* pattern2_;
public:

    inline Plane2DIndexPattern(IndexPattern* pattern1,
                               IndexPattern* pattern2) :
        pattern1_(pattern1),
        pattern2_(pattern2) {
        CPPADCG_ASSERT_UNKNOWN(pattern1_ != nullptr || pattern2_ != nullptr);
    }

    Plane2DIndexPattern(const Plane2DIndexPattern& orig) = delete;

    inline const IndexPattern* getPattern1() const {
        return pattern1_;
    }

    inline const IndexPattern* getPattern2() const {
        return pattern2_;
    }

    inline IndexPatternType getType() const override {
        return IndexPatternType::Plane2D;
    }

    inline void getSubIndexes(std::set<IndexPattern*>& indexes) const override {
        if (pattern1_ != nullptr) {
            indexes.insert(pattern1_);
            pattern1_->getSubIndexes(indexes);
        }
        if (pattern2_ != nullptr) {
            indexes.insert(pattern2_);
            pattern2_->getSubIndexes(indexes);
        }
    }

    inline virtual ~Plane2DIndexPattern() {
        delete pattern1_;
        delete pattern2_;
    }

    /***********************************************************************
     *                        static methods
     **********************************************************************/

    static inline Plane2DIndexPattern* detectPlane2D(const std::map<size_t, std::map<size_t, size_t> >& x2y2z) {
        /**
         * try to fit a combination of two patterns:
         *  z = fStart(x) + flit(y);
         */

        if (x2y2z.size() == 1) {
            // only one x -> fit z to y
            const std::map<size_t, size_t>& y2z = x2y2z.begin()->second;
            return new Plane2DIndexPattern(nullptr, IndexPattern::detect(y2z));
        }

        // perhaps there is always only one y
        size_t y = x2y2z.begin()->second.begin()->first;
        std::map<size_t, size_t> x2z;
        std::map<size_t, std::map<size_t, size_t> >::const_iterator itx2y2z;
        for (itx2y2z = x2y2z.begin(); itx2y2z != x2y2z.end(); ++itx2y2z) {
            size_t x = itx2y2z->first;
            const std::map<size_t, size_t>& y2z = itx2y2z->second;

            if (y2z.size() != 1 ||
                    y != y2z.begin()->first) {
                x2z.clear(); // not always the same y
                break;
            }

            size_t z = y2z.begin()->second;
            x2z[x] = z;
        }

        if (!x2z.empty()) {
            return new Plane2DIndexPattern(IndexPattern::detect(x2z), nullptr);
        }

        /**
         * try to fit a combination of two patterns:
         *  z = fStart(x) + flit(y);
         */
        std::map<size_t, size_t> x2zStart;
        std::map<size_t, size_t> y2zOffset;

        for (itx2y2z = x2y2z.begin(); itx2y2z != x2y2z.end(); ++itx2y2z) {
            size_t x = itx2y2z->first;
            const std::map<size_t, size_t>& y2z = itx2y2z->second;

            size_t zFirst = y2z.begin()->second;
            x2zStart[x] = zFirst;

            std::map<size_t, size_t>::const_iterator ity2z;

            for (ity2z = y2z.begin(); ity2z != y2z.end(); ++ity2z) {
                size_t y = ity2z->first;
                size_t offset = ity2z->second - zFirst;
                std::map<size_t, size_t>::const_iterator itY2zOffset = y2zOffset.find(y);
                if (itY2zOffset == y2zOffset.end()) {
                    y2zOffset[y] = offset;
                } else if (itY2zOffset->second != offset) {
                    return nullptr; // does not fit the pattern
                }
            }
        }

        /**
         * try to detect a pattern for the initial iteration index based on x
         */
        std::unique_ptr<IndexPattern> fx;

        std::map<size_t, IndexPattern*> startSections = SectionedIndexPattern::detectLinearSections(x2zStart, 2);
        if (startSections.empty()) {
            return nullptr; // does not fit the pattern
        }

        // detected a pattern for the first z based on x
        if (startSections.size() == 1) {
            fx = std::unique_ptr<IndexPattern> (startSections.begin()->second);
        } else {
            fx = std::unique_ptr<IndexPattern> (new SectionedIndexPattern(startSections));
        }

        /**
         * try to detect a pattern for the following iterations
         * based on the local loop index (local index != model index)
         */
        std::map<size_t, IndexPattern*> sections = SectionedIndexPattern::detectLinearSections(y2zOffset, 2);
        if (sections.empty()) {
            return nullptr; // does not fit the pattern
        }

        // detected a pattern for the z offset based on y
        std::unique_ptr<IndexPattern> fy;
        if (sections.size() == 1) {
            fy = std::unique_ptr<IndexPattern> (sections.begin()->second);
        } else {
            fy = std::unique_ptr<IndexPattern> (new SectionedIndexPattern(sections));
        }

        // simplify when both patterns are constant
        if (fx->getType() == IndexPatternType::Linear && fy->getType() == IndexPatternType::Linear) {
            LinearIndexPattern* ipx = static_cast<LinearIndexPattern*> (fx.get());
            LinearIndexPattern* ipy = static_cast<LinearIndexPattern*> (fy.get());

            if (ipx->getLinearSlopeDy() == 0 && ipy->getLinearSlopeDy() == 0) {
                /**
                 * only need to keep one
                 */
                ipx->setLinearConstantTerm(ipx->getLinearConstantTerm() + ipy->getLinearConstantTerm());
                fy.reset();
            }
        }

        return new Plane2DIndexPattern(fx.release(), fy.release());
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
