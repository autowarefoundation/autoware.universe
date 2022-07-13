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
#include "../models/distillation.hpp"


namespace CppAD {
namespace cg {

const size_t nStage = 8; // number of stages

class CppADCGPatternDistillationTest : public CppADCGPatternModelTest {
public:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
public:

    inline CppADCGPatternDistillationTest(bool verbose = false, bool printValues = false) :
        CppADCGPatternModelTest("Distillation",
                                nStage * 6, //ns
                                4, // nm
                                4, // npar
                                nStage * 6, // m
                                verbose, printValues) {

        // this->testZeroOrder_ = false;
        // this->testJacobian_ = false;
        // this->testHessian_ = false;
        this->verbose_ = true;
        //this->epsilonA_ = std::numeric_limits<Base>::epsilon() * 4e3;

#if 1
        /**
         * with normalization
         */
        xNorm_.resize(n, 1.0);

        size_t j = 0;
        for (size_t i = 0; i < nStage; i++, j++) xNorm_[j] = 12000; // mWater
        for (size_t i = 0; i < nStage; i++, j++) xNorm_[j] = 12000; // mEthanol[i]
        for (size_t i = 0; i < nStage; i++, j++) xNorm_[j] = 360; // T[i]
        for (size_t i = 0; i < nStage; i++, j++) xNorm_[j] = 0.5; // yWater[i]
        for (size_t i = 0; i < nStage; i++, j++) xNorm_[j] = 0.5; // yEthanol[i]
        for (size_t i = 0; i < nStage - 1; i++, j++) xNorm_[j] = 8; // V[i]
        xNorm_[j++] = 150e3; // Qc
        assert(j == ns);

        xNorm_[j++] = 250e3; // Qsteam 
        xNorm_[j++] = 0.1; // Fdistillate
        xNorm_[j++] = 2.5; // reflux
        xNorm_[j++] = 4; // Frectifier

        xNorm_[j++] = 30; // feed
        xNorm_[j++] = 1.01325e5; // P
        xNorm_[j++] = 0.7; // xFWater
        xNorm_[j++] = 366; // Tfeed

        xb = std::vector<Base>(n, 1.0);
        j = 0;
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = (12000 + 100 * i) / xNorm_[j]; // mWater
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = (12000 - 100 * i) / xNorm_[j]; // mEthanol[i]
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = (360 + i * 2) / xNorm_[j]; // T[i]
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = (0.3 + 0.05 * i) / xNorm_[j]; // yWater[i]
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = (0.7 - 0.05 * i) / xNorm_[j]; // yEthanol[i]
#else
        /**
         * without normalization 
         */
        size_t j = 0;
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = 12000 + 1; // mWater
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = 12000 + 1; // mEthanol[i]
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = 360 + (i + 1); // T[i]
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = 0.3 + 0.05 * i; // yWater[i]
        for (size_t i = 0; i < nStage; i++, j++) xb[j] = 0.7 - 0.05 * i; // yEthanol[i]
        for (size_t i = 0; i < nStage - 1; i++, j++) xb[j] = 8 + 0.1 * i; // V[i]
        xb[j++] = 150e3; // Qc
        assert(j == ns);

        xb[j++] = 250e3; // Qsteam 
        xb[j++] = 0.1; // Fdistillate
        xb[j++] = 2.5; // reflux
        xb[j++] = 4; // Frectifier

        xb[j++] = 30; // feed
        xb[j++] = 1.01325e5; // P
        xb[j++] = 0.7; // xFWater
        xb[j++] = 366; // Tfeed

#endif
    }

    virtual std::vector<ADCGD> evaluateModel(const std::vector<ADCGD>& x, size_t repeat) override {
        assert(repeat == nStage);
        return distillationFunc(x);
    }

    virtual std::vector<std::set<size_t> > getRelatedCandidates(size_t repeat) override {
        assert(repeat == nStage);
        
        std::vector<std::set<size_t> > relatedDepCandidates(6);
        size_t j = 0;
        for (size_t i = 0; i < nStage; i++, j++) relatedDepCandidates[0].insert(j); // mWater
        for (size_t i = 0; i < nStage; i++, j++) relatedDepCandidates[1].insert(j); // mEthanol
        for (size_t i = 0; i < nStage; i++, j++) relatedDepCandidates[4].insert(j); // T
        for (size_t i = 0; i < nStage; i++, j++) relatedDepCandidates[2].insert(j); // yWater
        for (size_t i = 0; i < nStage; i++, j++) relatedDepCandidates[3].insert(j); // yEthanol
        for (size_t i = 0; i < nStage - 1; i++, j++) relatedDepCandidates[5].insert(j); // V
        return relatedDepCandidates;
    }

};

}
}

using namespace CppAD;
using namespace CppAD::cg;

/**
 * @test test the usage of loops for the generation of distillation model
 */
TEST_F(CppADCGPatternDistillationTest, distillationAllVars) {
    modelName += "AllVars";
    useCustomSparsity_ = false;

    this->test(nStage);
}

TEST_F(CppADCGPatternDistillationTest, distillation) {
    useCustomSparsity_ = true;

    this->test(nStage);
}
