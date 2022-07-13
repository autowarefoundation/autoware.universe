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

using namespace CppAD;
using namespace CppAD::cg;

TEST_F(CppADCGTest, std_vector) {
    std::vector<double> x(4);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = i + 1;

    CppAD::cg::ArrayView<double> y(x);

    ASSERT_EQ(x.size(), y.size());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], y[i]);
        ASSERT_EQ(x[i], y.at(i));
    }

}


TEST_F(CppADCGTest, std_vector_const) {
    std::vector<double> x(4);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = i + 1;

    CppAD::cg::ArrayView<const double> y(x);

    ASSERT_EQ(x.size(), y.size());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], y[i]);
        ASSERT_EQ(x[i], y.at(i));
    }

}

TEST_F(CppADCGTest, array) {
    std::array<double, 4> x;
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = i + 1;

    CppAD::cg::ArrayView<double> y(x);

    ASSERT_EQ(x.size(), y.size());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], y[i]);
        ASSERT_EQ(x[i], y.at(i));
    }
}

TEST_F(CppADCGTest, array_const) {
    std::array<double, 4> x;
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = i + 1;

    CppAD::cg::ArrayView<const double> y(x);

    ASSERT_EQ(x.size(), y.size());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], y[i]);
        ASSERT_EQ(x[i], y.at(i));
    }
}

TEST_F(CppADCGTest, valarray) {
    std::valarray<double> x(4);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = i + 1;

    CppAD::cg::ArrayView<double> y(x);

    ASSERT_EQ(x.size(), y.size());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], y[i]);
        ASSERT_EQ(x[i], y.at(i));
    }
}

TEST_F(CppADCGTest, valarray_const) {
    std::valarray<double> x(4);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = i + 1;

    CppAD::cg::ArrayView<const double> y(x);

    ASSERT_EQ(x.size(), y.size());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], y[i]);
        ASSERT_EQ(x[i], y.at(i));
    }
}

TEST_F(CppADCGTest, copy_std_vector) {
    std::vector<double> x(4);
    std::vector<double> z(4);
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] = i + 1;
        z[i] = i + 10;
    }

    CppAD::cg::ArrayView<double> y(x);

    ASSERT_EQ(x.size(), y.size());

    y = z;

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], z[i]);
    }
}

TEST_F(CppADCGTest, copy_cppad_vector) {
    std::vector<double> x(4);
    CppAD::vector<double> z(4);
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] = i + 1;
        z[i] = i + 10;
    }

    CppAD::cg::ArrayView<double> y(x);

    ASSERT_EQ(x.size(), y.size());

    y = z;

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], z[i]);
    }
}

TEST_F(CppADCGTest, copy_array) {
    std::vector<double> x(4);
    std::array<double, 4> z;
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] = i + 1;
        z[i] = i + 10;
    }

    CppAD::cg::ArrayView<double> y(x);

    ASSERT_EQ(x.size(), y.size());

    y = z;

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], z[i]);
    }
}

TEST_F(CppADCGTest, copy_valarray) {
    std::vector<double> x(4);
    std::valarray<double> z(4);
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] = i + 1;
        z[i] = i + 10;
    }

    CppAD::cg::ArrayView<double> y(x);

    ASSERT_EQ(x.size(), y.size());

    y = z;

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(x[i], z[i]);
    }
}

TEST_F(CppADCGTest, copy_arrayview) {
    std::vector<double> x(4);
    std::vector<double> z(4);
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] = i + 1;
        z[i] = i + 10;
    }

    CppAD::cg::ArrayView<double> xx(x);
    CppAD::cg::ArrayView<double> zz(z);

    xx = zz;

    ASSERT_TRUE(xx.data() !=  zz.data());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(xx[i], zz[i]);
    }
}

TEST_F(CppADCGTest, copy_arrayview_const) {
    std::vector<double> x(4);
    std::vector<double> z(4);
    for (size_t i = 0; i < x.size(); ++i) {
        x[i] = i + 1;
        z[i] = i + 10;
    }

    CppAD::cg::ArrayView<double> xx(x);
    CppAD::cg::ArrayView<const double> zz(z);

    xx = zz;

    ASSERT_TRUE(xx.data() !=  zz.data());

    for(size_t i = 0; i < x.size(); ++i) {
        ASSERT_EQ(xx[i], zz[i]);
    }
}
