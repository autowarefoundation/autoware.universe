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

using Base = double;
using CGD = CppAD::cg::CG<Base>;
using ADCGD = CppAD::AD<CGD>;

namespace CppAD {
namespace cg {

class CppADCGHessLoopTest : public CppADCGPatternTest {
public:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
public:

    inline CppADCGHessLoopTest(bool verbose = false, bool printValues = false) :
        CppADCGPatternTest(verbose, printValues) {
        this->testJacobian_ = false;
    }
};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD;
using namespace CppAD::cg;

/**
 * @test Linear model (no Hessian)
 */
std::vector<ADCGD> modelLinear(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = 3 * x[i * n];
        y[i * m + 1] = x[i * n + 1] + x[i * n];
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelLinear) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelLinear);
    testLibCreation("modelLinear", m, n, 6);
}

/**
 * @test Bi-linear model of indexed variables
 */
std::vector<ADCGD> modelBiLinearIndexed(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = 3 * x[i * n];
        y[i * m + 1] = x[i * n + 1] * x[i * n];
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelBiLinearIndexed) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelBiLinearIndexed);
    testLibCreation("modelBiLinearIndexed", m, n, 6);
}

/**
 * @test Bi-linear model of indexed-nonIndexed variables
 */
std::vector<ADCGD> modelBiLinearIndexedNonIndexed(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = 3 * x[i * n];
        y[i * m + 1] = x[i * n + 1] * x[0];
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelBiLinearIndexedNonIndexed) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelBiLinearIndexedNonIndexed);
    testLibCreation("modelBiLinearIndexedNonIndexed", m, n, 6);
}

/**
 * @test Model with a bilinear term with a temporary (which depends linearly
 *       on a non-indexed) and an indexed variable
 */
std::vector<ADCGD> modelIndexedTemporary(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp = 5 * x[0];
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = 3 * x[i * n];
        y[i * m + 1] = x[i * n + 1] * tmp;
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelIndexedTemporary) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelIndexedTemporary);
    testLibCreation("modelIndexedTemporary", m, n, 6);
}

/**
 * @test Model with two temporary variables (which depend linearly on non-indexed)
 */
std::vector<ADCGD> modelTemporaryTemporary(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = 5 * x[0];
    ADCGD tmp2 = 4 * x[2];
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = 3 * tmp1 * x[i * n] * tmp2;
        y[i * m + 1] = x[i * n + 1];
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporaryTemporary) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelTemporaryTemporary);
    testLibCreation("modelTemporaryTemporary", m, n, 6);
}

/**
 * @test Model with a non-indexed and a temporary variable (which depends
 *       linearly on a non-indexed)
 */
std::vector<ADCGD> modelTemporaryNonIndexed(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp = 4 * x[2];
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = x[0] * x[i * n] * tmp;
        y[i * m + 1] = x[i * n + 1];
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporaryNonIndexed) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelTemporaryNonIndexed);
    testLibCreation("modelTemporaryNonIndexed", m, n, 6);
}

/**
 * @test Model with a temporary variable (which depends non-linearly on a
 *       non-indexed)
 */
std::vector<ADCGD> modelTemporary(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp = 4 * x[2] * x[0];
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = tmp;
        y[i * m + 1] = x[i * n + 1];
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporary) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelTemporary);
    testLibCreation("modelTemporary", m, n, 6);
}

/**
 * @test Model with bilinear term with a temporary variable (which depends 
 *       non-linearly on a non-indexed) and an indexed variable
 */
std::vector<ADCGD> modelTemporaryIndexed(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 1;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp = cos(x[2]);
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = x[i * n] * tmp;
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporaryIndexed) {
    size_t m = 1;
    size_t n = 2;

    setModel(modelTemporaryIndexed);
    testLibCreation("modelTemporaryIndexed", m, n, 6);
}

/**
 * @test Model with 2 bilinear term with a temporary variable (which depends 
 *       non-linearly on a non-indexed) and an indexed variable
 */
std::vector<ADCGD> modelTemporaryIndexed2(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = cos(x[2]);
    ADCGD tmp2 = cos(x[0]);
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = x[i * n] * tmp1;
        y[i * m + 1] = x[i * n + 1] * tmp2;
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporaryIndexed2) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelTemporaryIndexed2);
    testLibCreation("modelTemporaryIndexed2", m, n, 6);
}

/**
 * @test Model with a temporary variable (which depends non-linearly on a 
 *       non-indexed) and a non-linear term with an indexed variable
 */
std::vector<ADCGD> modelTemporaryIndexed3(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 1;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp = cos(x[0]);
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = x[i * n + 1] / x[i * n] * tmp;
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporaryIndexed3) {
    size_t m = 1;
    size_t n = 2;

    setModel(modelTemporaryIndexed3);
    testLibCreation("modelTemporaryIndexed3", m, n, 6);
}

/**
 * @test Model with two temporary variables (which depends non-linearly on the
 *       same non-indexed variable) and an indexed variable
 */
std::vector<ADCGD> modelTemporaryIndexed4(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 1;
    size_t n = 1;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = 2 * x[0];
    ADCGD tmp2 = 3 * x[0];
    ADCGD tmp3 = 4 * x[0];
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = tmp1 * x[i * n] + x[i * n] * tmp2 + x[i * n] * tmp3;
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporaryIndexed4) {
    size_t m = 1;
    size_t n = 1;

    setModel(modelTemporaryIndexed4);
    testLibCreation("modelTemporaryIndexed4", m, n, 6);
}

/**
 * @test Model with 2 equations each with a bilinear term with a temporary 
 *       variable (which depends non-linearly on a non-indexed) and an indexed
 *       variable. Tests the grouping of expressions in the same if-else branch
 */
std::vector<ADCGD> modelTemporaryIndexed5(const std::vector<ADCGD>& x, size_t repeat) {
    size_t m = 2;
    size_t n = 2;
    size_t m2 = repeat * m;

    // dependent variable vector 
    std::vector<ADCGD> y(m2);

    ADCGD tmp1 = cos(x[0]);
    ADCGD tmp2 = sin(x[1]);
    for (size_t i = 0; i < repeat; i++) {
        y[i * m] = x[i * n] * tmp1;
        y[i * m + 1] = x[i * n + 1] * tmp2;
    }

    return y;
}

TEST_F(CppADCGHessLoopTest, modelTemporaryIndexed5) {
    size_t m = 2;
    size_t n = 2;

    setModel(modelTemporaryIndexed5);
    testLibCreation("modelTemporaryIndexed5", m, n, 6);
}

/**
 * @test example model
 */
std::vector<ADCGD> modelExample(const std::vector<ADCGD>& x, size_t repeat) {
    assert(repeat == 3);

    // dependent variable vector 
    std::vector<ADCGD> y(8);

    // temporary variables
    ADCGD a, b;

    // the model    
    a = exp(3 * x[1]);

    b = 5 * x[0] * x[4];
    y[0] = a / 2 + b;
    // one equation not defined!
    y[1] = x[2] - b;

    b = 5 * x[1] * x[3];
    y[2] = a / 2 + b;
    y[3] = x[4] * x[1] + b;
    y[4] = x[3] - b;

    b = 5 * x[2] * x[2];
    y[5] = a / 2 + b;
    y[6] = x[4] * x[2] + b;
    y[7] = x[4] - b;

    return y;
}

TEST_F(CppADCGHessLoopTest, modelExample) {
    size_t repeat = 3;

    std::vector<std::set<size_t> > relatedDeps{
        {0, 2, 5}, // equation pattern 1
        {3, 6}, // equation pattern 2
        {1, 4, 7} // equation pattern 3
    };
    std::vector<std::vector<std::set<size_t> > > loops{relatedDeps}; // one loop

    setModel(modelExample);

    std::vector<Base> xb(5, 0.5);
    testPatternDetection(xb, repeat, relatedDeps, loops);

    testLibCreation("modelExample", relatedDeps, repeat, xb);
}
