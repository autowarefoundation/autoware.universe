/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2017 Ciengis
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
#include "CppADCGTest.hpp"
#include "cppad/cg/support/cppadcg_eigen.hpp"

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGTest, EigenCGD) {
    using Eigen::Dynamic;
    using Eigen::Matrix;

    using CGD = CG<double>;

    using traits = Eigen::NumTraits<CGD>;

    ASSERT_EQ(traits::IsComplex, 0);
    ASSERT_EQ(traits::IsInteger, 0);
    ASSERT_EQ(traits::IsSigned, 1);
    ASSERT_EQ(traits::RequireInitialization, 1);
    ASSERT_EQ(traits::ReadCost, 1);
    ASSERT_EQ(traits::AddCost, 2);
    ASSERT_EQ(traits::MulCost, 2);

    ASSERT_EQ(traits::epsilon(), CppAD::numeric_limits<CGD>::epsilon());
    ASSERT_EQ(traits::dummy_precision(), 100. * CppAD::numeric_limits<CGD>::epsilon());
    ASSERT_EQ(traits::highest(), CppAD::numeric_limits<CGD>::max());
    ASSERT_EQ(traits::lowest(), CppAD::numeric_limits<CGD>::min());

    CGD x = 2.0;
    ASSERT_EQ(conj(x), x);
    ASSERT_EQ(real(x), x);
    ASSERT_EQ(imag(x), 0.0);
    ASSERT_EQ(abs2(x), 4.0);

    // Outputing a matrix
    Matrix<CGD, 1, 1> X;
    X(0, 0) = 1;
    std::stringstream stream_out;
    stream_out << X;
    ASSERT_EQ("1", stream_out.str());

    // multiplying three matrices together.
    Matrix<CGD, Dynamic, Dynamic> A(1, 1), B(1, 1), C(1, 1), D(1, 1);
    A(0, 0) = 1.0;
    B(0, 0) = 2.0;
    C(0, 0) = 3.0;
    D = A * B * C;
    ASSERT_EQ(D(0, 0), 6.0);

    // multiplying matrix and scalar.
    D = 2.0 * D;
    ASSERT_EQ(D(0, 0), 12.0);

    C = C * 2.0;
    ASSERT_EQ(C(0, 0), 6.0);
}


TEST_F(CppADCGTest, EigenADCGD) {
    using CppAD::AD;
    using Eigen::Dynamic;
    using Eigen::Matrix;

    using CGD = CG<double>;
    using ADCGD = AD<CGD>;

    using traits = Eigen::NumTraits<ADCGD>;

    ASSERT_EQ(traits::IsComplex, 0);
    ASSERT_EQ(traits::IsInteger, 0);
    ASSERT_EQ(traits::IsSigned, 1);
    ASSERT_EQ(traits::RequireInitialization, 1);
    ASSERT_EQ(traits::ReadCost, 1);
    ASSERT_EQ(traits::AddCost, 2);
    ASSERT_EQ(traits::MulCost, 2);

    ASSERT_EQ(traits::epsilon(), std::numeric_limits<double>::epsilon());
    ASSERT_EQ(traits::dummy_precision(), 100. * std::numeric_limits<double>::epsilon());
    ASSERT_EQ(traits::highest(), (std::numeric_limits<double>::max)());
    ASSERT_EQ(traits::lowest(), (std::numeric_limits<double>::min)());

    ADCGD x = CGD(2.0);
    ASSERT_EQ(conj(x), x);
    ASSERT_EQ(real(x), x);
    ASSERT_EQ(imag(x), 0.0);
    ASSERT_EQ(abs2(x), 4.0);

    // Outputing a matrix
    Matrix<ADCGD, 1, 1> X;
    X(0, 0) = ADCGD(1);
    std::stringstream stream_out;
    stream_out << X;
    ASSERT_EQ("1", stream_out.str());

    // multiplying three matrices
    Matrix<ADCGD, Dynamic, Dynamic> A(1, 1), B(1, 1), C(1, 1), D(1, 1);
    A(0, 0) = 1.0;
    B(0, 0) = 2.0;
    C(0, 0) = 3.0;

    D = A * B * C;
    ASSERT_EQ(D(0, 0), 6.0);

    // multiplying matrix and scalar.
    D = 2.0 * C;
    ASSERT_EQ(D(0, 0), 6.0);

    D = C * 2.0;
    ASSERT_EQ(D(0, 0), 6.0);

    D = CGD(2.0) * C;
    ASSERT_EQ(D(0, 0), 6.0);

    D = C * CGD(2.0);
    ASSERT_EQ(D(0, 0), 6.0);
}
