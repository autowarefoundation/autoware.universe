/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
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
#include <stdlib.h>
#include <math.h>
#include <cppad/cg/model/threadpool/pthread_pool.h>
#include "CppADCGTest.hpp"

extern "C" {

void pooldynamic_sparse_reverse_one_dep0(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
    //independent variables
    const double* x = in[0];
    const double* py = in[1];

    //dependent variables
    double* dw = out[0];

    // auxiliary variables

    dw[0] = (0 - sin(x[0])) * py[0];
}

void pooldynamic_sparse_reverse_one_dep1(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
    //independent variables
    const double* x = in[0];
    const double* py = in[1];

    //dependent variables
    double* dw = out[0];

    // auxiliary variables

    dw[0] = cos(x[0]) * py[0];
    dw[1] = x[2] * py[0];
    dw[2] = x[1] * py[0];
}

void pooldynamic_sparse_reverse_one_dep2(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
    //independent variables
    const double* x = in[0];
    const double* py = in[1];

    //dependent variables
    double* dw = out[0];

    // auxiliary variables

    dw[0] = (0 - sin(x[3])) * py[0];
}

void pooldynamic_sparse_reverse_one_dep3(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
    //independent variables
    const double* x = in[0];
    const double* py = in[1];

    //dependent variables
    double* dw = out[0];

    // auxiliary variables

    dw[0] = cos(x[3]) * py[0];
    dw[1] = x[5] * py[0];
    dw[2] = x[4] * py[0];
}

void pooldynamic_sparse_reverse_one_dep4(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
    //independent variables
    const double* x = in[0];
    const double* py = in[1];

    //dependent variables
    double* dw = out[0];

    // auxiliary variables

    dw[0] = (0 - sin(x[6])) * py[0];
}

void pooldynamic_sparse_reverse_one_dep5(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
    //independent variables
    const double* x = in[0];
    const double* py = in[1];

    //dependent variables
    double* dw = out[0];

    // auxiliary variables

    dw[0] = cos(x[6]) * py[0];
    dw[1] = x[8] * py[0];
    dw[2] = x[7] * py[0];
}


typedef void (* cppadcg_function_type)(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun);

typedef struct ExecArgStruct {
    cppadcg_function_type func;
    double const* const* in;
    double* out[1];
    struct LangCAtomicFun atomicFun;
} ExecArgStruct;

static void exec_func(void* arg) {
    ExecArgStruct* eArg = (ExecArgStruct*) arg;
    (*eArg->func)(eArg->in, eArg->out, eArg->atomicFun);
}

void pooldynamic_sparse_jacobian(double const* const* in, double* const* out, struct LangCAtomicFun atomicFun) {
    static const cppadcg_function_type p[6] = {pooldynamic_sparse_reverse_one_dep0, pooldynamic_sparse_reverse_one_dep1, pooldynamic_sparse_reverse_one_dep2,
                                               pooldynamic_sparse_reverse_one_dep3, pooldynamic_sparse_reverse_one_dep4, pooldynamic_sparse_reverse_one_dep5};
    static const long offset[6] = {0, 1, 4, 5, 8, 9};
    double const* inLocal[2];
    double inLocal1 = 1;
    //double * outLocal[1];
    double* jac = out[0];
    long i;

    inLocal[0] = in[0];
    inLocal[1] = &inLocal1;

    ExecArgStruct* args[6];
    static cppadcg_thpool_function_type execute_functions[6] = {exec_func, exec_func, exec_func, exec_func, exec_func, exec_func};
    static float avgElapsed[6] = {0, 0, 0, 0, 0, 0};
    float elapsed[6] = {0, 0, 0, 0, 0, 0};
    static int order[6] = {0, 1, 2, 3, 4, 5};
    static int job2Thread[6] = {-1, -1, -1, -1, -1, -1};
    static int lastElapsedChanged = 1;
    unsigned int nBench = cppadcg_thpool_get_n_time_meas();
    static unsigned int meas = 0;
    int do_benchmark = (meas < nBench && !cppadcg_thpool_is_disabled());
    float* elapsed_p = do_benchmark ? elapsed : NULL;

    for (i = 0; i < 6; ++i) {
        args[i] = (ExecArgStruct*) malloc(sizeof(ExecArgStruct));
        args[i]->func = p[i];
        args[i]->in = inLocal;
        args[i]->out[0] = &jac[offset[i]];
        args[i]->atomicFun = atomicFun;
    }

    cppadcg_thpool_add_jobs(execute_functions, (void**) args, avgElapsed, elapsed_p, order, job2Thread, 6, lastElapsedChanged);

    cppadcg_thpool_wait();

    for (i = 0; i < 6; ++i) {
        free(args[i]);
    }

    if (do_benchmark) {
        cppadcg_thpool_update_order(avgElapsed, meas, elapsed, order, 6);
        meas++;
    } else {
        lastElapsedChanged = 0;
    }

}

}

namespace CppAD {
namespace cg {

class PThreadPoolTest : public CppADCGTest {
    using CGD = CG<double>;
    using ADCG = AD<CGD>;
protected:
    struct LangCAtomicFun atomicFun;
    std::vector<double> in0;
    std::vector<double const*> in;
    std::vector<double> out0;
    std::vector<double*> out;
    std::vector<double> jac;
public:

    inline PThreadPoolTest(bool verbose = true) :
            in0(9),
            in{in0.data()},
            out0(12),
            out{out0.data()},
            jac(12) {
        cppadcg_thpool_set_verbose(1);
        cppadcg_thpool_set_n_time_meas(5);

        jac[0] = -0.99749498660405445;
        jac[1] = 0.070737201667702906;
        jac[2] = 1.5;
        jac[3] = 1.5;
        jac[4] = -0.99749498660405445;
        jac[5] = 0.070737201667702906;
        jac[6] = 1.5;
        jac[7] = 1.5;
        jac[8] = -0.99749498660405445;
        jac[9] = 0.070737201667702906;
        jac[10] = 1.5;
        jac[11] = 1.5;

        for (auto& ui : in0)
            ui = 1.5;
    }

    virtual void TearDown() override {
        cppadcg_thpool_shutdown();
    }
};

} // END cg namespace
} // END CppAD namespace

using namespace CppAD::cg;

TEST_F(PThreadPoolTest, DynamicJac) {
    cppadcg_thpool_set_scheduler_strategy(SCHED_DYNAMIC);

    pooldynamic_sparse_jacobian(in.data(), out.data(), atomicFun);

    pooldynamic_sparse_jacobian(in.data(), out.data(), atomicFun);

    ASSERT_TRUE(compareValues(jac, out0));
}

TEST_F(PThreadPoolTest, GuidedJac) {
    cppadcg_thpool_set_scheduler_strategy(SCHED_GUIDED);

    pooldynamic_sparse_jacobian(in.data(), out.data(), atomicFun);

    pooldynamic_sparse_jacobian(in.data(), out.data(), atomicFun);

    ASSERT_TRUE(compareValues(jac, out0));
}

TEST_F(PThreadPoolTest, StaticJac) {
    cppadcg_thpool_set_scheduler_strategy(SCHED_STATIC);

    pooldynamic_sparse_jacobian(in.data(), out.data(), atomicFun); // last elapsed time measurements

    pooldynamic_sparse_jacobian(in.data(), out.data(), atomicFun); // last work group schedule update

    pooldynamic_sparse_jacobian(in.data(), out.data(), atomicFun); // reuse previous work group schedule

    ASSERT_TRUE(compareValues(jac, out0));
}