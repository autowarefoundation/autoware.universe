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

#include <omp.h>
#include <stdio.h>

enum ScheduleStrategy {SCHED_STATIC = 1,
                       SCHED_DYNAMIC = 2,
                       SCHED_GUIDED = 3
                      };

static volatile int cppadcg_openmp_enabled = 1; // false
static volatile int cppadcg_openmp_verbose = 1; // false
static volatile unsigned int cppadcg_openmp_n_threads = 2;

static enum ScheduleStrategy schedule_strategy = SCHED_DYNAMIC;


void cppadcg_openmp_set_disabled(int disabled) {
    cppadcg_openmp_enabled = !disabled;
}

int cppadcg_openmp_is_disabled() {
    return !cppadcg_openmp_enabled;
}

void cppadcg_openmp_set_verbose(int v) {
    cppadcg_openmp_verbose = v;
}

int cppadcg_openmp_is_verbose() {
    return cppadcg_openmp_verbose;
}

void cppadcg_openmp_set_threads(unsigned int n) {
    if(n <= 0)
        n = 1;
    cppadcg_openmp_n_threads = n;
}

unsigned int cppadcg_openmp_get_threads() {
    return cppadcg_openmp_n_threads;
}

void cppadcg_openmp_set_scheduler_strategy(enum ScheduleStrategy s) {
    schedule_strategy = s;
}

enum ScheduleStrategy cppadcg_openmp_get_scheduler_strategy() {
    return schedule_strategy;
}

void cppadcg_openmp_apply_scheduler_strategy() {
    if (schedule_strategy == SCHED_DYNAMIC) {
        omp_set_schedule(omp_sched_dynamic, 1);
    } else if (schedule_strategy == SCHED_GUIDED) {
        omp_set_schedule(omp_sched_guided, 0);
    } else {
        omp_set_schedule(omp_sched_static, 0);
    }
}