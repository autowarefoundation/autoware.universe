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
#include "../models/tank_battery.hpp"

namespace CppAD {
namespace cg {

const size_t nTanks = 6; // number of stages

class CppADCGPatternTankBatTest : public CppADCGPatternModelTest {
public:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
public:

    inline CppADCGPatternTankBatTest(bool verbose = false, bool printValues = false) :
        CppADCGPatternModelTest("TankBattery",
                                nTanks, // ns
                                1, // nm
                                1, // npar
                                nTanks * 1, // m
                                verbose, printValues) {
        this->verbose_ = true;
    }

    virtual std::vector<ADCGD> evaluateModel(const std::vector<ADCGD>& x, size_t repeat) {
        assert(repeat == nTanks);
        return tankBatteryFunc(x);
    }

    virtual std::vector<std::set<size_t> > getRelatedCandidates(size_t repeat) {
        std::vector<std::set<size_t> > relatedDepCandidates(1);
        for (size_t i = 0; i < repeat; i++) relatedDepCandidates[0].insert(i);
        return relatedDepCandidates;
    }

};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;

/**
 * @test test the usage of loops for the generation of the tank battery model
 *       with the creation of differential information for all variables
 */
TEST_F(CppADCGPatternTankBatTest, tankBatteryAllVars) {
    modelName += "AllVars";

    useCustomSparsity_ = false;

    /**
     * test
     */
    this->test(6);
}

/**
 * @test test the usage of loops for the generation of the tank battery model
 *       with the creation of differential information only for states and
 *       controls
 */
TEST_F(CppADCGPatternTankBatTest, tankBattery) {
    useCustomSparsity_ = true;

    /**
     * test
     */
    this->test(6);
}