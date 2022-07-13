/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2019 Joao Leal
 *    Copyright (C) 2016 Ciengis
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
#include "ThreadPoolTest.hpp"

using namespace CppAD::cg;

namespace CppAD {
namespace cg {

class CppADCGThreadPoolDisabledTest : public ThreadPoolTest {
public:
    explicit CppADCGThreadPoolDisabledTest() :
            ThreadPoolTest(MultiThreadingType::PTHREADS) {
        this->_multithreadDisabled = true;
    }
};

} // END cg namespace
} // END CppAD namespace

TEST_F(CppADCGThreadPoolDisabledTest, ForwardZero) {
    this->testForwardZero();
}

TEST_F(CppADCGThreadPoolDisabledTest, Jacobian) {
    this->testJacobian();
}

TEST_F(CppADCGThreadPoolDisabledTest, Hessian) {
    this->testHessian();
}

namespace CppAD {
namespace cg {

class CppADCGThreadPoolDynamicTest : public ThreadPoolTest {
public:
    explicit CppADCGThreadPoolDynamicTest() :
            ThreadPoolTest(MultiThreadingType::PTHREADS) {
        this->_multithreadDisabled = false;
        this->_multithreadScheduler = ThreadPoolScheduleStrategy::DYNAMIC;
    }
};

} // END cg namespace
} // END CppAD namespace

TEST_F(CppADCGThreadPoolDynamicTest, ForwardZero) {
    this->testForwardZero();
}

TEST_F(CppADCGThreadPoolDynamicTest, Jacobian) {
    this->testJacobian();
}

TEST_F(CppADCGThreadPoolDynamicTest, Hessian) {
    this->testHessian();
}

namespace CppAD {
namespace cg {

class CppADCGThreadPoolGuidedTest : public ThreadPoolTest {
public:
    explicit CppADCGThreadPoolGuidedTest() :
            ThreadPoolTest(MultiThreadingType::PTHREADS) {
        this->_multithreadDisabled = false;
        this->_multithreadScheduler = ThreadPoolScheduleStrategy::GUIDED;
    }
};

} // END cg namespace
} // END CppAD namespace

TEST_F(CppADCGThreadPoolGuidedTest, ForwardZero) {
    this->testForwardZero();
}

TEST_F(CppADCGThreadPoolGuidedTest, Jacobian) {
    this->testJacobian();
}

TEST_F(CppADCGThreadPoolGuidedTest, Hessian) {
    this->testHessian();
}

namespace CppAD {
namespace cg {

class CppADCGThreadPoolStaticTest : public ThreadPoolTest {
public:
    explicit CppADCGThreadPoolStaticTest() :
            ThreadPoolTest(MultiThreadingType::PTHREADS) {
        this->_multithreadDisabled = false;
        this->_multithreadScheduler = ThreadPoolScheduleStrategy::STATIC;
    }
};

} // END cg namespace
} // END CppAD namespace

TEST_F(CppADCGThreadPoolStaticTest, ForwardZero) {
    this->testForwardZero();
}

TEST_F(CppADCGThreadPoolStaticTest, Jacobian) {
    this->testJacobian();
}

TEST_F(CppADCGThreadPoolStaticTest, Hessian) {
    this->testHessian();
}

namespace CppAD {
namespace cg {

class CppADCGThreadPoolDynamicCustomTest : public ThreadPoolTest {
public:
    explicit CppADCGThreadPoolDynamicCustomTest() :
            ThreadPoolTest(MultiThreadingType::PTHREADS) {
        this->_multithreadDisabled = false;
        this->_multithreadScheduler = ThreadPoolScheduleStrategy::DYNAMIC;

        // all elements except 1
        _jacRow = {0, 1, 1};
        _jacCol = {0, 0, 2};

        // all elements except 1
        _hessRow = {0, 2};
        _hessCol = {0, 1};
    }
};

} // END cg namespace
} // END CppAD namespace

TEST_F(CppADCGThreadPoolDynamicCustomTest, ForwardZero) {
    this->testForwardZero();
}

TEST_F(CppADCGThreadPoolDynamicCustomTest, Jacobian) {
    this->testJacobian();
}

TEST_F(CppADCGThreadPoolDynamicCustomTest, Hessian) {
    this->testHessian();
}
