/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
#ifndef CPPAD_CG_CPPADCGTEST_HPP
#define CPPAD_CG_CPPADCGTEST_HPP

#include <vector>
#include <valarray>

#include <gtest/gtest.h>

#include <cppad/cg.hpp>
#include <cppad/utility/memory_leak.hpp>
#include <cppad/utility/near_equal.hpp>

namespace CppAD {
namespace cg {

class CppADCGTest : public ::testing::Test {
protected:
    using Base = double;
    using CGD = CppAD::cg::CG<Base>;
    using ADCGD = CppAD::AD<CGD>;
    bool verbose_;
    bool printValues_;
    bool memory_check_;
public:

    inline explicit CppADCGTest(bool verbose = false,
                                bool printValues = false) :
        verbose_(verbose),
        printValues_(printValues),
        memory_check_(true) {
    }

    void TearDown() override {
        if (memory_check_) {
            ASSERT_FALSE(CppAD::memory_leak());
        }
    }

protected:

    static inline ::testing::AssertionResult compareValues(const std::vector<Base>& depCGen,
                                                           const std::vector<CppAD::cg::CG<Base> >& dep,
                                                           double epsilonR = 1e-14, double epsilonA = 1e-14) {

        std::vector<double> depd(dep.size());

        for (size_t i = 0; i < depd.size(); i++) {
            depd[i] = dep[i].getValue();
        }

        return compareValues<Base>(depCGen, depd, epsilonR, epsilonA);
    }

    template<class T>
    static inline ::testing::AssertionResult compareValues(const std::vector<std::vector<T> >& depCGen,
                                                           const std::vector<std::vector<T> >& dep,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {

        assert(depCGen.size() == dep.size());

        for (size_t i = 0; i < depCGen.size(); i++) {
            ::testing::AssertionResult r = compareValues<T>(depCGen[i], dep[i], epsilonR, epsilonA);
            if (!r) {
                return ::testing::AssertionFailure() << "Comparison failed for array " << i << ":\n" << r.failure_message();
            }
        }

        return ::testing::AssertionSuccess();
    }

    template<class T, class T2>
    static inline ::testing::AssertionResult compareValues(const std::vector<T>& cgen,
                                                           const std::vector<T2>& orig,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {
        return compareValues(ArrayView<const T>(cgen),
                             ArrayView<const T2>(orig),
                             epsilonR, epsilonA);
    }

    template<class T, class T2>
    static inline ::testing::AssertionResult compareValues(const CppAD::vector<T>& cgen,
                                                           const CppAD::vector<T2>& orig,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {
        return compareValues(ArrayView<const T>(cgen),
                             ArrayView<const T2>(orig),
                             epsilonR, epsilonA);
    }

    template<class T, class T2>
    static inline ::testing::AssertionResult compareValues(const std::valarray<T>& cgen,
                                                           const std::valarray<T2>& orig,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {
        return compareValues(ArrayView<const T>(cgen),
                             ArrayView<const T2>(orig),
                             epsilonR, epsilonA);
    }

    template<class T, class T2>
    static inline ::testing::AssertionResult compareValues(ArrayView<const T> cgen,
                                                           ArrayView<const T2> orig,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {
        if (cgen.size() != orig.size())
            return ::testing::AssertionFailure() << "Dimensions do not match (" << cgen.size() << " != " << orig.size()
                                                 << ").";

        std::ostringstream ss;
        for (size_t i = 0; i < cgen.size(); i++) {
            ::testing::AssertionResult r = nearEqual(cgen[i], orig[i], epsilonR, epsilonA);
            if (!r) {
                ss << "Failed at element " << i << ": " << r.failure_message() << "\n";
            }
        }

        if (ss.str().empty())
            return ::testing::AssertionSuccess();
        else
            return ::testing::AssertionFailure() << ss.str();
    }

    template<class T>
    static inline ::testing::AssertionResult compareValues(const CppAD::vector<T>& depCGen,
                                                           const CppAD::vector<CppAD::cg::CG<T> >& dep,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {
        return compareValues(ArrayView<const T>(depCGen),
                             ArrayView<const CppAD::cg::CG<T> >(dep),
                             epsilonR, epsilonA);
    }

    template<class T>
    static inline ::testing::AssertionResult compareValues(const std::vector<T>& depCGen,
                                                           const std::vector<CppAD::cg::CG<T> >& dep,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {
        return compareValues(ArrayView<const T>(depCGen),
                             ArrayView<const CppAD::cg::CG<T> >(dep),
                             epsilonR, epsilonA);
    }

    template<class T>
    static inline ::testing::AssertionResult compareValues(const std::vector<T>& depCGen,
                                                           const CppAD::vector<CppAD::cg::CG<T> >& dep,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {
        return compareValues(ArrayView<const T>(depCGen),
                             ArrayView<const CppAD::cg::CG<T> >(dep),
                             epsilonR, epsilonA);
    }

    template<class T>
    static inline ::testing::AssertionResult compareValues(ArrayView<const T> depCGen,
                                                           ArrayView<const CppAD::cg::CG<T> > dep,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {

        std::vector<T> depd(dep.size());

        for (size_t i = 0; i < depd.size(); i++) {
            depd[i] = dep[i].getValue();
        }

        return compareValues<T>(depCGen, ArrayView<const T>(depd), epsilonR, epsilonA);
    }

    template<class T>
    static inline ::testing::AssertionResult compareValues(ArrayView<const CppAD::cg::CG<T> > dep1,
                                                           ArrayView<const CppAD::cg::CG<T> > dep2,
                                                           T epsilonR = std::numeric_limits<T>::epsilon() * 100,
                                                           T epsilonA = std::numeric_limits<T>::epsilon() * 100) {

        std::vector<T> dep1d(dep1.size());

        for (size_t i = 0; i < dep1d.size(); i++) {
            dep1d[i] = dep1[i].getValue();
        }

        std::vector<T> dep2d(dep2.size());

        for (size_t i = 0; i < dep2d.size(); i++) {
            dep2d[i] = dep2[i].getValue();
        }

        return compareValues<T>(ArrayView<const T>(dep2d), ArrayView<const T>(dep1d), epsilonR, epsilonA);
    }


    template<class VectorBool>
    static inline void compareBoolValues(const VectorBool& expected,
                                         const VectorBool& value) {
        ASSERT_EQ(expected.size(), value.size());
        for (size_t i = 0; i < expected.size(); i++) {
            ASSERT_EQ(expected[i], value[i]);
        }
    }

    template<class VectorSet>
    static inline void compareVectorSetValues(const VectorSet& expected,
                                              const VectorSet& value) {
        ASSERT_EQ(expected.size(), value.size());
        for (size_t i = 0; i < expected.size(); i++) {
            ASSERT_EQ(expected[i].size(), value[i].size());
            auto itE = expected[i].begin();
            auto itV = value[i].begin();
            for (; itE != expected[i].end(); ++itE, ++itV) {
                ASSERT_EQ(*itE, *itV);
            }
        }
    }

    template <class T>
    static inline ::testing::AssertionResult nearEqual(const T &x, const T &y,
                                                       const T &r = std::numeric_limits<T>::epsilon() * 100,
                                                       const T &a = std::numeric_limits<T>::epsilon() * 100) {

        T zero(0);
        if (r <= zero)
            return ::testing::AssertionFailure() << "Invalid relative tolerance. Must be positive!";
        if (a <= zero)
            return ::testing::AssertionFailure() << "Invalid absolute tolerance. Must be positive!";

        // check for special cases
        T infinity = T(1) / zero;

        // NaN case
        if (x != x)
            return ::testing::AssertionFailure() << "NaN value found!";
        if (y != y)
            return ::testing::AssertionFailure() << "NaN value found!";

        // infinite cases
        if (x == infinity || x == -infinity)
            return ::testing::AssertionFailure() << "Infinity value found!";
        if (y == infinity || y == -infinity)
            return ::testing::AssertionFailure() << "Infinity value found!";

        T ax = x;
        if (ax < zero)
            ax = -ax;

        T ay = y;
        if (ay < zero)
            ay = -ay;

        T ad = x - y;
        if (ad < zero)
            ad = -ad;

        if (ad > a) {
            T rel = r * (ax + ay);
            if (ad > rel) {// compare like this to avoid problems when ax and ay are zero
                return ::testing::AssertionFailure() << "Absolute error (" << ad << ") is higher than tolerance (" << a << ")"
                        "\n       and relative error (" << (ad / (ax + ay)) << ") is higher than tolerance (" << r << ")"
                        "\n       for the values " << x << " and " << y;
            }
        }

        return ::testing::AssertionSuccess();
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
