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
#include "CppADCGPatternModelTest.hpp"
#include "../models/plug_flow2.hpp"


namespace CppAD {
namespace cg {

const size_t nEls = 6; // number of discretization elements

class CppADCGPatternPlugFlowTest : public CppADCGPatternModelTest {
public:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
public:

    inline CppADCGPatternPlugFlowTest(bool verbose = false, bool printValues = false) :
        CppADCGPatternModelTest("PlugFlow",
                                4 * nEls, // ns
                                1, // nm
                                4, // npar
                                nEls * 4, // m
                                verbose, printValues) {
        this->verbose_ = false;

        //this->epsilonA_ = std::numeric_limits<Base>::epsilon() * 1e2; 
        //this->hessianEpsilonA_ = std::numeric_limits<Base>::epsilon() * 1e2; 
        //this->hessianEpsilonR_ = std::numeric_limits<Base>::epsilon() * 1e2; 

        this->xb = PlugFlowModel2<CGD>::getTypicalValues(nEls);
    }

    virtual std::vector<ADCGD> evaluateModel(const std::vector<ADCGD>& x, size_t repeat) {
        PlugFlowModel2<CGD> m;

        assert(repeat == nEls);
        return m.model(x);
    }

    virtual std::vector<std::set<size_t> > getRelatedCandidates(size_t nEls) {
        return PlugFlowModel2<CGD>::getRelatedCandidates(nEls);
    }

    inline virtual void defineCustomSparsity(ADFun<CGD>& fun) {
        CppADCGPatternModelTest::defineCustomSparsity(fun);

        //std::vector<std::set<size_t> > hessSparAll = hessianSparsitySet<std::vector<std::set<size_t> > >(fun);
        //printSparsityPattern(hessSparAll, "Full Hessian");
        //customHessSparsity_.resize(x.size());
        //customHessSparsity_[22].insert(22);
    }

};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;

/**
 * @test test the usage of loops for the generation of the plug flow model
 *       with the creation of differential information for all variables
 */
TEST_F(CppADCGPatternPlugFlowTest, plugflowAllVars) {
    modelName += "AllVars";

    this->useCustomSparsity_ = false;

    this->test(nEls);
}

/**
 * @test test the usage of loops for the generation of the plug flow model
 *       with the creation of differential information only for states and
 *       controls
 */
TEST_F(CppADCGPatternPlugFlowTest, plugflow) {
    this->useCustomSparsity_ = true;

    this->test(nEls);
}