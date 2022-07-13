#ifndef CPPAD_CG_TEST_CPPADCGPATTERNMODELTEST_INCLUDED
#define	CPPAD_CG_TEST_CPPADCGPATTERNMODELTEST_INCLUDED
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
#include "CppADCGPatternTest.hpp"

namespace CppAD {
namespace cg {

class CppADCGPatternModelTest : public CppADCGPatternTest, public PatternTestModel<CG<double> > {
public:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
protected:
    std::string modelName;
    const size_t ns;
    const size_t nm;
    const size_t m; // total number of equations in the model
    const size_t n; // number of independent variables
    std::vector<Base> xb; // values for the model
    bool useCustomSparsity_;
public:

    inline CppADCGPatternModelTest(const std::string& modelName_,
                                   const size_t ns_,
                                   const size_t nm_,
                                   const size_t npar_,
                                   const size_t m_,
                                   bool verbose = false,
                                   bool printValues = false) :
        CppADCGPatternTest(verbose, printValues),
        modelName(modelName_),
        ns(ns_),
        nm(nm_),
        m(m_),
        n(ns_ + nm_ + npar_),
        xb(n),
        useCustomSparsity_(true) {
        for (size_t j = 0; j < n; j++)
            xb[j] = 0.5 * (j + 1);

        setModel(*this);
    }

    virtual std::vector<std::set<size_t> > getRelatedCandidates(size_t repeat) = 0;

    void test(size_t repeat) {

        std::vector<std::set<size_t> > relatedDepCandidates = getRelatedCandidates(repeat);

        testLibCreation("model" + modelName, relatedDepCandidates, repeat, xb);
    }

    void testPatterns(size_t repeat, const std::vector<std::vector<std::set<size_t> > >& loops) {
        std::vector<std::set<size_t> > relatedDepCandidates = getRelatedCandidates(repeat);

        this->testPatternDetection(xb, repeat, relatedDepCandidates, loops);
    }


protected:

    inline virtual void defineCustomSparsity(ADFun<CGD>& fun) {
        if (!useCustomSparsity_)
            return;

        /**
         * Determine the relevant elements
         */
        std::vector<std::set<size_t> > jacSparAll = jacobianSparsitySet<std::vector<std::set<size_t> > >(fun);
        customJacSparsity_.resize(jacSparAll.size());
        for (size_t i = 0; i < jacSparAll.size(); i++) {
            // only differential information for states and controls
            std::set<size_t>::const_iterator itEnd = jacSparAll[i].upper_bound(ns + nm - 1);
            if (itEnd != jacSparAll[i].begin())
                customJacSparsity_[i].insert(jacSparAll[i].begin(), itEnd);
        }

        std::vector<std::set<size_t> > hessSparAll = hessianSparsitySet<std::vector<std::set<size_t> > >(fun);
        customHessSparsity_.resize(hessSparAll.size());
        for (size_t i = 0; i < ns + nm; i++) {
            std::set<size_t>::const_iterator it = hessSparAll[i].upper_bound(i); // only the lower left side
            if (it != hessSparAll[i].begin())
                customHessSparsity_[i].insert(hessSparAll[i].begin(), it);
        }
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
