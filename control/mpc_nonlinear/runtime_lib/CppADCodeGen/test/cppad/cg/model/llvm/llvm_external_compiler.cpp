/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
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

#include "LlvmModelTest.hpp"

using namespace CppAD;
using namespace CppAD::cg;

class LlvmModelExternalCompilerTest : public LlvmModelTest {
public:
    std::unique_ptr<LlvmModelLibrary<Base> > compileLib(LlvmModelLibraryProcessor<double>& p) override {
        ClangCompiler<double> clang("/usr/bin/clang-" + p.getVersion());
        return p.create(clang);
    }
};


TEST_F(LlvmModelExternalCompilerTest, ForwardZero) {
    testForwardZeroResults(*model, *fun, nullptr, x);
}

TEST_F(LlvmModelExternalCompilerTest, DenseJacobian) {
    testDenseJacResults(*model, *fun, x);
}

TEST_F(LlvmModelExternalCompilerTest, DenseHessian) {
    testDenseHessianResults(*model, *fun, x);
}

TEST_F(LlvmModelExternalCompilerTest, Jacobian) {
    // sparse Jacobian again (make sure the second run is also OK)
    size_t n_tests = llvmModelLib->getThreadNumber() > 1 ? 2 : 1;

    testSparseJacobianResults(n_tests, *model, *fun, nullptr, x, false);
}

TEST_F(LlvmModelExternalCompilerTest, Hessian) {
    // sparse Hessian again (make sure the second run is also OK)
    size_t n_tests = llvmModelLib->getThreadNumber() > 1 ? 2 : 1;

    testSparseHessianResults(n_tests, *model, *fun, nullptr, x, false);
}
