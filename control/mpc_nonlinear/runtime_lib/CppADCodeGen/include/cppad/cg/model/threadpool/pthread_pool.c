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
 * Authors: Johan Hanssen Seferidis, Joao Leal
 */

/**
 * This file was adapted by Joao Leal from
 *  https://github.com/Pithikos/C-Thread-Pool/blob/master/thpool.c
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#if defined(__linux__)
#include <sys/prctl.h>
#include <time.h>
#include <sys/time.h>
#define __USE_GNU /* required before including  resource.h */
#include <sys/resource.h>
#endif

enum ScheduleStrategy {SCHED_STATIC = 1,
                       SCHED_DYNAMIC = 2,
                       SCHED_GUIDED = 3
                       };

enum ElapsedTimeReference {ELAPSED_TIME_AVG,
                           ELAPSED_TIME_MIN};

typedef struct ThPool ThPool;
typedef void (* thpool_function_type)(void*);

static ThPool* volatile cppadcg_pool = NULL;
static int cppadcg_pool_n_threads = 2;
static int cppadcg_pool_disabled = 0; // false
static int cppadcg_pool_verbose = 0; // false
static enum ElapsedTimeReference cppadcg_pool_time_update = ELAPSED_TIME_MIN;
static unsigned int cppadcg_pool_time_meas = 10; // default number of time measurements
static float cppadcg_pool_guided_maxgroupwork = 0.75;

static enum ScheduleStrategy schedule_strategy = SCHED_DYNAMIC;

/* ==================== INTERNAL HIGH LEVEL API  ====================== */

static ThPool* thpool_init(int num_threads);

static int thpool_add_job(ThPool*,
                          thpool_function_type function,
                          void* arg,
                          const float* avgElapsed,
                          float* elapsed);

static int thpool_add_jobs(ThPool*,
                           thpool_function_type functions[],
                           void* args[],
                           const float avgElapsed[],
                           float elapsed[],
                           const int order[],
                           int job2Thread[],
                           int nJobs,
                           int lastElapsedChanged);

static void thpool_wait(ThPool*);

static void thpool_destroy(ThPool*);

/* ========================== STRUCTURES ============================ */
/* Binary semaphore */
typedef struct BSem {
    pthread_mutex_t mutex;
    pthread_cond_t   cond;
    int v;
} BSem;


/* Job */
typedef struct Job {
    struct Job*  prev;                   /* pointer to previous job              */
    thpool_function_type function;       /* function pointer                     */
    void*  arg;                          /* function's argument                  */
    const float* avgElapsed;             /* the last measurement of elapsed time */
    float* elapsed;                      /* the current elapsed time             */
    struct timespec startTime;           /* initial time (verbose only)          */
    struct timespec endTime;             /* final time (verbose only)            */
    int id;                              /* a job identifier used for debugging  */
} Job;

/* Work group */
typedef struct WorkGroup {
    struct WorkGroup*  prev;             /* pointer to previous WorkGroup  */
    struct Job* jobs;                    /* jobs                           */
    int size;                            /* number of jobs                 */
    struct timespec startTime;           /* initial time (verbose only)    */
    struct timespec endTime;             /* final time (verbose only)      */
} WorkGroup;

/* Job queue */
typedef struct JobQueue {
    pthread_mutex_t rwmutex;             /* used for queue r/w access */
    Job  *front;                         /* pointer to front of queue */
    Job  *rear;                          /* pointer to rear  of queue */
    WorkGroup* group_front;              /* previously created work groups (SCHED_STATIC scheduling only)*/
    BSem *has_jobs;                      /* flag as binary semaphore  */
    int   len;                           /* number of jobs in queue   */
    float total_time;                    /* total expected time to complete the work */
    float highest_expected_return;       /* the time when the last running thread is expected to request new work */
} JobQueue;


/* Thread */
typedef struct Thread {
    int id;                              /* friendly id                          */
    pthread_t pthread;                   /* pointer to actual thread             */
    struct ThPool* thpool;               /* access to ThPool                     */
    WorkGroup* processed_groups;         /* processed work groups (verbose only) */
} Thread;


/* Threadpool */
typedef struct ThPool {
    Thread** threads;                    /* pointer to threads        */
    int num_threads;                     /* total number of threads   */
    volatile int num_threads_alive;      /* threads currently alive   */
    volatile int num_threads_working;    /* threads currently working */
    pthread_mutex_t thcount_lock;        /* used for thread count etc */
    pthread_cond_t threads_all_idle;     /* signal to thpool_wait     */
    JobQueue* jobqueue;                  /* pointer to the job queue  */
    volatile int threads_keepalive;
} ThPool;

/* ========================== PUBLIC API ============================ */

void cppadcg_thpool_set_threads(int n) {
    cppadcg_pool_n_threads = n;
}

int cppadcg_thpool_get_threads() {
    return cppadcg_pool_n_threads;
}

void cppadcg_thpool_set_scheduler_strategy(enum ScheduleStrategy s) {
    if(cppadcg_pool != NULL) {
        pthread_mutex_lock(&cppadcg_pool->jobqueue->rwmutex);
        schedule_strategy = s;
        pthread_mutex_unlock(&cppadcg_pool->jobqueue->rwmutex);
    } else {
        // pool not yet created
        schedule_strategy = s;
    }
}

enum ScheduleStrategy cppadcg_thpool_get_scheduler_strategy() {
    if(cppadcg_pool != NULL) {
        enum ScheduleStrategy e;
        pthread_mutex_lock(&cppadcg_pool->jobqueue->rwmutex);
        e = schedule_strategy;
        pthread_mutex_unlock(&cppadcg_pool->jobqueue->rwmutex);
        return e;
    } else {
        // pool not yet created
        return schedule_strategy;
    }
}

void cppadcg_thpool_set_disabled(int disabled) {
    cppadcg_pool_disabled = disabled;
}

int cppadcg_thpool_is_disabled() {
    return cppadcg_pool_disabled;
}

void cppadcg_thpool_set_guided_maxgroupwork(float v) {
    if(cppadcg_pool != NULL) {
        pthread_mutex_lock(&cppadcg_pool->jobqueue->rwmutex);
        cppadcg_pool_guided_maxgroupwork = v;
        pthread_mutex_unlock(&cppadcg_pool->jobqueue->rwmutex);
    } else {
        // pool not yet created
        cppadcg_pool_guided_maxgroupwork = v;
    }
}

float cppadcg_thpool_get_guided_maxgroupwork() {
    if(cppadcg_pool != NULL) {
        float r;
        pthread_mutex_lock(&cppadcg_pool->jobqueue->rwmutex);
        r = cppadcg_pool_guided_maxgroupwork;
        pthread_mutex_unlock(&cppadcg_pool->jobqueue->rwmutex);
        return r;
    } else {
        // pool not yet created
        return cppadcg_pool_guided_maxgroupwork;
    }
}

unsigned int cppadcg_thpool_get_n_time_meas() {
    return cppadcg_pool_time_meas;
}

void cppadcg_thpool_set_n_time_meas(unsigned int n) {
    cppadcg_pool_time_meas = n;
}

void cppadcg_thpool_set_verbose(int v) {
    cppadcg_pool_verbose = v;
}

enum ElapsedTimeReference cppadcg_thpool_get_time_meas_ref() {
    return cppadcg_pool_time_update;
}

void cppadcg_thpool_set_time_meas_ref(enum ElapsedTimeReference r) {
    cppadcg_pool_time_update = r;
}

int cppadcg_thpool_is_verbose() {
    return cppadcg_pool_verbose;
}

void cppadcg_thpool_prepare() {
    if(cppadcg_pool == NULL) {
        cppadcg_pool = thpool_init(cppadcg_pool_n_threads);
    }
}

void cppadcg_thpool_add_job(thpool_function_type function,
                            void* arg,
                            float* avgElapsed,
                            float* elapsed) {
    if (!cppadcg_pool_disabled) {
        cppadcg_thpool_prepare();
        if (cppadcg_pool != NULL) {
            thpool_add_job(cppadcg_pool, function, arg, avgElapsed, elapsed);
            return;
        }
    }

    // thread pool not used
    (*function)(arg);
}

void cppadcg_thpool_add_jobs(thpool_function_type functions[],
                             void* args[],
                             const float avgElapsed[],
                             float elapsed[],
                             const int order[],
                             int job2Thread[],
                             int nJobs,
                             int lastElapsedChanged) {
    int i;
    if (!cppadcg_pool_disabled) {
        cppadcg_thpool_prepare();
        if (cppadcg_pool != NULL) {
            thpool_add_jobs(cppadcg_pool, functions, args, avgElapsed, elapsed, order, job2Thread, nJobs, lastElapsedChanged);
            return;
        }
    }

    // thread pool not used
    for (i = 0; i < nJobs; ++i) {
        (*functions[i])(args[i]);
    }
}

void cppadcg_thpool_wait() {
    if(cppadcg_pool != NULL) {
        thpool_wait(cppadcg_pool);
    }
}

typedef struct pair_double_int {
    float val;
    int index;
} pair_double_int;

static int comparePair(const void* a, const void* b) {
    if (((pair_double_int*) a)->val < ((pair_double_int*) b)->val)
        return -1;
    if (((pair_double_int*) a)->val == ((pair_double_int*) b)->val)
        return 0;
    return 1;
}

void cppadcg_thpool_update_order(float refElapsed[],
                                 unsigned int nTimeMeas,
                                 const float elapsed[],
                                 int order[],
                                 int nJobs) {
    if(nJobs == 0 || refElapsed == NULL || elapsed == NULL || order == NULL)
        return;

    struct pair_double_int elapsedOrder[nJobs];
    int i;
    int nonZero = 0; // false

    for(i = 0; i < nJobs; ++i) {
        if(elapsed[i] != 0) {
            nonZero = 1;
            break;
        }
    }

    if (!nonZero) {
        if (cppadcg_pool_verbose) {
            fprintf(stdout, "order not updated: all times are zero\n");
        }
        return;
    }

    if(cppadcg_pool_time_update == ELAPSED_TIME_AVG) {
        for (i = 0; i < nJobs; ++i) {
            refElapsed[i] = (refElapsed[i] * nTimeMeas + elapsed[i]) / (nTimeMeas + 1);
            elapsedOrder[i].val = refElapsed[i];
            elapsedOrder[i].index = i;
        }
    } else {
        // cppadcg_pool_time_update == ELAPSED_TIME_MIN
        for (i = 0; i < nJobs; ++i) {
            if(nTimeMeas == 0 || elapsed[i] < refElapsed[i]) {
                refElapsed[i] = elapsed[i];
            }
            elapsedOrder[i].val = refElapsed[i];
            elapsedOrder[i].index = i;
        }
    }

    qsort(elapsedOrder, nJobs, sizeof(struct pair_double_int), comparePair);

    for (i = 0; i < nJobs; ++i) {
        order[elapsedOrder[i].index] = nJobs - i - 1; // descending order
    }

    if (cppadcg_pool_verbose) {
        fprintf(stdout, "new order (%i values):\n", nTimeMeas + 1);
        for (i = 0; i < nJobs; ++i) {
            fprintf(stdout, " job id: %i   order: %i   time: %e s\n", i, order[i], refElapsed[i]);
        }
    }

}

void cppadcg_thpool_shutdown() {
    if(cppadcg_pool != NULL) {
        thpool_destroy(cppadcg_pool);
        cppadcg_pool = NULL;
    }
}

/* ========================== PROTOTYPES ============================ */

static void thpool_cleanup(ThPool* thpool);

static int  thread_init(ThPool* thpool,
                        Thread** thread,
                        int id);
static void* thread_do(Thread* thread);
static void  thread_destroy(Thread* thread);

static int   jobqueue_init(ThPool* thpool);
static void  jobqueue_clear(ThPool* thpool);
static void  jobqueue_push(JobQueue* queue,
                           Job* newjob_p);
static void jobqueue_multipush(JobQueue* queue,
                               Job* newjob[],
                               int nJobs);
static int jobqueue_push_static_jobs(ThPool* thpool,
                                     Job* newjobs[],
                                     const float avgElapsed[],
                                     int jobs2thread[],
                                     int nJobs,
                                     int lastElapsedChanged);
static WorkGroup* jobqueue_pull(ThPool* thpool, int id);
static void  jobqueue_destroy(ThPool* thpool);

static void  bsem_init(BSem *bsem, int value);
static void  bsem_reset(BSem *bsem);
static void  bsem_post(BSem *bsem);
static void  bsem_post_all(BSem *bsem);
static void  bsem_wait(BSem *bsem);


/* ============================ TIME ============================== */

static float get_thread_time(struct timespec* cputime,
                             int* info) {
    *info = clock_gettime(CLOCK_THREAD_CPUTIME_ID, cputime);
    if(*info == 0) {
        return cputime->tv_sec + cputime->tv_nsec * 1e-9f;
    } else {
        fprintf(stderr, "failed clock_gettime()\n");
        return 0;
    }
}

static float get_monotonic_time(struct timespec* time,
                                int* info) {
    *info = clock_gettime(CLOCK_MONOTONIC, time);
    if(*info == 0) {
        return time->tv_sec + time->tv_nsec * 1e-9f;
    } else {
        fprintf(stderr, "failed clock_gettime()\n");
        return 0;
    }
}

static void get_monotonic_time2(struct timespec* time) {
    int info;
    info = clock_gettime(CLOCK_MONOTONIC, time);
    if(info != 0) {
        time->tv_sec = 0;
        time->tv_nsec = 0;
        fprintf(stderr, "failed clock_gettime()\n");
    }
}

void timespec_diff(struct timespec* end,
                   struct timespec* start,
                   struct timespec* result) {
    if ((end->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = end->tv_sec - start->tv_sec - 1;
        result->tv_nsec = end->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = end->tv_sec - start->tv_sec;
        result->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
}

/* ========================== THREADPOOL ============================ */

/**
 * @brief  Initialize threadpool
 *
 * Initializes a threadpool. This function will not return untill all
 * threads have initialized successfully.
 *
 * @example
 *
 *    ..
 *    threadpool thpool;                     //First we declare a threadpool
 *    thpool = thpool_init(4);               //then we initialize it to 4 threads
 *    ..
 *
 * @param  num_threads   number of threads to be created in the threadpool
 * @return threadpool    created threadpool on success,
 *                       NULL on error
 */
struct ThPool* thpool_init(int num_threads) {
    if (num_threads < 0) {
        num_threads = 0;
    }

    if(cppadcg_pool_verbose) {
        fprintf(stdout, "thpool_init(): Thread pool created with %i threads\n", num_threads);
    }

    if(num_threads == 0) {
        cppadcg_pool_disabled = 1; // true
        return NULL;
    }

    /* Make new thread pool */
    ThPool* thpool;
    thpool = (ThPool*) malloc(sizeof(ThPool));
    if (thpool == NULL) {
        fprintf(stderr, "thpool_init(): Could not allocate memory for thread pool\n");
        return NULL;
    }
    thpool->num_threads = num_threads;
    thpool->num_threads_alive = 0;
    thpool->num_threads_working = 0;
    thpool->threads_keepalive = 1;

    /* Initialize the job queue */
    if (jobqueue_init(thpool) == -1) {
        fprintf(stderr, "thpool_init(): Could not allocate memory for job queue\n");
        free(thpool);
        return NULL;
    }

    /* Make threads in pool */
    thpool->threads = (Thread**) malloc(num_threads * sizeof(Thread*));
    if (thpool->threads == NULL) {
        fprintf(stderr, "thpool_init(): Could not allocate memory for threads\n");
        jobqueue_destroy(thpool);
        free(thpool->jobqueue);
        free(thpool);
        return NULL;
    }

    pthread_mutex_init(&(thpool->thcount_lock), NULL);
    pthread_cond_init(&thpool->threads_all_idle, NULL);

    /* Thread init */
    int n;
    for (n = 0; n < num_threads; n++) {
        thread_init(thpool, &thpool->threads[n], n);
    }

    /* Wait for threads to initialize */
    while (thpool->num_threads_alive != num_threads) {}

    return thpool;
}

/**
 * @brief Add work to the job queue
 *
 * Takes an action and its argument and adds it to the threadpool's job queue.
 * If you want to add to work a function with more than one arguments then
 * a way to implement this is by passing a pointer to a structure.
 *
 * NOTICE: You have to cast both the function and argument to not get warnings.
 *
 * @example
 *
 *    void print_num(int num){
 *       printf("%d\n", num);
 *    }
 *
 *    int main() {
 *       ..
 *       int a = 10;
 *       thpool_add_job(thpool, (void*)print_num, (void*)a);
 *       ..
 *    }
 *
 * @param  threadpool    threadpool to which the work will be added
 * @param  function      pointer to function to add as work
 * @param  arg           pointer to an argument
 * @return 0 on successs, -1 otherwise.
 */
static int thpool_add_job(ThPool* thpool,
                          thpool_function_type function,
                          void* arg,
                          const float* avgElapsed,
                          float* elapsed) {
    Job* newjob;

    newjob = (struct Job*) malloc(sizeof(struct Job));
    if (newjob == NULL) {
        fprintf(stderr, "thpool_add_job(): Could not allocate memory for new job\n");
        return -1;
    }

    /* add function and argument */
    newjob->function = function;
    newjob->arg = arg;
    newjob->avgElapsed = avgElapsed;
    newjob->elapsed = elapsed;

    /* add job to queue */
    jobqueue_push(thpool->jobqueue, newjob);

    return 0;
}

static int thpool_add_jobs(ThPool* thpool,
                           thpool_function_type functions[],
                           void* args[],
                           const float avgElapsed[],
                           float elapsed[],
                           const int order[],
                           int job2Thread[],
                           int nJobs,
                           int lastElapsedChanged) {
    Job* newjobs[nJobs];
    int i;
    int j;

    for (i = 0; i < nJobs; ++i) {
        newjobs[i] = (Job*) malloc(sizeof(Job));
        if (newjobs[i] == NULL) {
            fprintf(stderr, "thpool_add_jobs(): Could not allocate memory for new jobs\n");
            return -1;
        }

        j = order != NULL ? order[i] : i;
        /* add function and argument */
        newjobs[i]->function = functions[j];
        newjobs[i]->arg = args[j];
        newjobs[i]->id = i;
        if (avgElapsed != NULL)
            newjobs[i]->avgElapsed = &avgElapsed[j];
        else
            newjobs[i]->avgElapsed = NULL;

        if (elapsed != NULL)
            newjobs[i]->elapsed = &elapsed[j];
        else
            newjobs[i]->elapsed = NULL;
    }

    /* add jobs to queue */
    if (schedule_strategy == SCHED_STATIC && avgElapsed != NULL && order != NULL && nJobs > 0 && avgElapsed[0] > 0) {
        return jobqueue_push_static_jobs(thpool, newjobs, avgElapsed, job2Thread, nJobs, lastElapsedChanged);
    } else {
        jobqueue_multipush(thpool->jobqueue, newjobs, nJobs);
        return 0;
    }
}

/**
 * Split work among the threads evenly considering the elapsed time of each job.
 */
static int jobqueue_push_static_jobs(ThPool* thpool,
                                     Job* newjobs[],
                                     const float avgElapsed[],
                                     int jobs2thread[],
                                     int nJobs,
                                     int lastElapsedChanged) {
    float total_duration, target_duration, next_duration, best_duration;
    int i, j, iBest;
    int added;
    int num_threads = thpool->num_threads;
    int* n_jobs;
    float* durations = NULL;
    WorkGroup** groups;
    WorkGroup* group;

    if(nJobs < num_threads)
        num_threads = nJobs;

    n_jobs = (int*) malloc(num_threads * sizeof(int));
    if (n_jobs == NULL) {
        fprintf(stderr, "jobqueue_push_static_jobs(): Could not allocate memory\n");
        return -1;
    }

    groups = (WorkGroup**) malloc(num_threads * sizeof(WorkGroup*));
    if (groups == NULL) {
        fprintf(stderr, "jobqueue_push_static_jobs(): Could not allocate memory\n");
        free(n_jobs);
        return -1;
    }

    for (i = 0; i < num_threads; ++i) {
        n_jobs[i] = 0;
    }

    total_duration = 0;
    for (i = 0; i < nJobs; ++i) {
        total_duration += avgElapsed[i];
    }


    if (nJobs > 0 && (lastElapsedChanged || jobs2thread[0] < 0)) {
        durations = (float*) malloc(num_threads * sizeof(float));
        if (durations == NULL) {
            fprintf(stderr, "jobqueue_push_static_jobs(): Could not allocate memory\n");
            free(n_jobs);
            free(groups);
            return -1;
        }

        for(i = 0; i < num_threads; ++i) {
            durations[i] = 0;
        }

        // decide in which work group to place each job
        target_duration = total_duration / num_threads;

        for (j = 0; j < nJobs; ++j) {
            added = 0;
            for (i = 0; i < num_threads; ++i) {
                next_duration = durations[i] + avgElapsed[j];
                if (next_duration < target_duration) {
                    durations[i] = next_duration;
                    n_jobs[i]++;
                    jobs2thread[j] = i;
                    added = 1;
                    break;
                }
            }

            if (!added) {
                best_duration = durations[0] + avgElapsed[j];
                iBest = 0;
                for (i = 1; i < num_threads; ++i) {
                    next_duration = durations[i] + avgElapsed[j];
                    if (next_duration < best_duration) {
                        best_duration = next_duration;
                        iBest = i;
                    }
                }
                durations[iBest] = best_duration;
                n_jobs[iBest]++;
                jobs2thread[j] = iBest;
            }
        }

    } else {
        // reuse existing information

        for (j = 0; j < nJobs; ++j) {
            n_jobs[jobs2thread[j]]++;
        }
    }

    /**
     * create the work groups
     */
    for (i = 0; i < num_threads; ++i) {
        group = (WorkGroup*) malloc(sizeof(WorkGroup));
        group->size = 0;
        group->jobs = (Job*) malloc(n_jobs[i] * sizeof(Job));
        groups[i] = group;
    }
    for (i = 0; i < num_threads - 1; ++i) {
        groups[i]->prev = groups[i + 1];
    }
    groups[num_threads - 1]->prev = NULL;

    // place jobs on the work groups
    for (j = 0; j < nJobs; ++j) {
        i = jobs2thread[j];
        group = groups[i];
        group->jobs[group->size] = *newjobs[j]; // copy
        group->size++;
        free(newjobs[j]);
    }

    if (cppadcg_pool_verbose) {
        if (durations != NULL) {
            for (i = 0; i < num_threads; ++i) {
                fprintf(stdout, "jobqueue_push_static_jobs(): work group %i with %i jobs for %e s\n", i, groups[i]->size, durations[i]);
            }
        } else {
            for (i = 0; i < num_threads; ++i) {
                fprintf(stdout, "jobqueue_push_static_jobs(): work group %i with %i jobs\n", i, groups[i]->size);
            }
        }
    }

    /**
     * add to the queue
     */
    pthread_mutex_lock(&thpool->jobqueue->rwmutex);

    groups[num_threads - 1]->prev = thpool->jobqueue->group_front;
    thpool->jobqueue->group_front = groups[0];

    bsem_post_all(thpool->jobqueue->has_jobs);

    pthread_mutex_unlock(&thpool->jobqueue->rwmutex);

    // clean up
    free(durations);
    free(n_jobs);
    free(groups);

    return 0;
}

/**
 * @brief Wait for all queued jobs to finish
 *
 * Will wait for all jobs - both queued and currently running to finish.
 * Once the queue is empty and all work has completed, the calling thread
 * (probably the main program) will continue.
 *
 * Smart polling is used in wait. The polling is initially 0 - meaning that
 * there is virtually no polling at all. If after 1 seconds the threads
 * haven't finished, the polling interval starts growing exponentially
 * untill it reaches max_secs seconds. Then it jumps down to a maximum polling
 * interval assuming that heavy processing is being used in the threadpool.
 *
 * @example
 *
 *    ..
 *    threadpool thpool = thpool_init(4);
 *    ..
 *    // Add a bunch of work
 *    ..
 *    thpool_wait(thpool);
 *    puts("All added work has finished");
 *    ..
 *
 * @param threadpool     the threadpool to wait for
 */
static void thpool_wait(ThPool* thpool) {
    pthread_mutex_lock(&thpool->thcount_lock);
    while (thpool->jobqueue->len || thpool->jobqueue->group_front || thpool->num_threads_working) {  //// PROBLEM HERE!!!! len is not locked!!!!
        pthread_cond_wait(&thpool->threads_all_idle, &thpool->thcount_lock);
    }
    thpool->jobqueue->total_time = 0;
    thpool->jobqueue->highest_expected_return = 0;
    pthread_mutex_unlock(&thpool->thcount_lock);

    thpool_cleanup(thpool);
}


/**
 * Called to clean-up after waiting for a thread pool to end the current work.
 * It is only required when  cppadcg_pool_verbose  was enabled.
 *
 * @param thpool
 */
void thpool_cleanup(ThPool* thpool) {
    // for debugging only

    struct timespec diffTime;
    int gid = 0;
    Thread* thread;
    WorkGroup* workGroup;
    WorkGroup* workGroupPrev;

    if (!cppadcg_pool_verbose) {
        return;
    }

    for (int j = 0; j < thpool->num_threads; ++j) {
        thread = thpool->threads[j];

        workGroup = thread->processed_groups;
        while (workGroup != NULL) {
            timespec_diff(&workGroup->endTime, &workGroup->startTime, &diffTime);
            fprintf(stdout, "# Thread %i, Group %i, started at %ld.%.9ld, ended at %ld.%.9ld, elapsed %ld.%.9ld, executed %i jobs\n",
                    thread->id, gid, workGroup->startTime.tv_sec, workGroup->startTime.tv_nsec, workGroup->endTime.tv_sec, workGroup->endTime.tv_nsec, diffTime.tv_sec,
                    diffTime.tv_nsec, workGroup->size);

            for (int i = 0; i < workGroup->size; ++i) {
                Job* job = &workGroup->jobs[i];

                timespec_diff(&job->endTime, &job->startTime, &diffTime);
                fprintf(stdout, "## Thread %i, Group %i, Job %i, started at %ld.%.9ld, ended at %ld.%.9ld, elapsed %ld.%.9ld\n",
                        thread->id, gid, job->id, job->startTime.tv_sec, job->startTime.tv_nsec, job->endTime.tv_sec, job->endTime.tv_nsec, diffTime.tv_sec,
                        diffTime.tv_nsec);
            }

            gid++;

            workGroupPrev = workGroup->prev;

            // clean-up
            free(workGroup->jobs);
            free(workGroup);

            workGroup = workGroupPrev;
        }

        thread->processed_groups = NULL;
    }
}

/**
 * @brief Destroy the threadpool
 *
 * This will wait for the currently active threads to finish and then 'kill'
 * the whole threadpool to free up memory.
 *
 * @example
 * int main() {
 *    threadpool thpool1 = thpool_init(2);
 *    threadpool thpool2 = thpool_init(2);
 *    ..
 *    thpool_destroy(thpool1);
 *    ..
 *    return 0;
 * }
 *
 * @param threadpool     the threadpool to destroy
 * @return nothing
 */
static void thpool_destroy(ThPool* thpool) {
    /* No need to destory if it's NULL */
    if (thpool == NULL) return;

    volatile int threads_total = thpool->num_threads_alive;

    /* End each thread 's infinite loop */
    thpool->threads_keepalive = 0;

    /* Give one second to kill idle threads */
    double TIMEOUT = 1.0;
    time_t start, end;
    double tpassed = 0.0;
    time(&start);
    while (tpassed < TIMEOUT && thpool->num_threads_alive) {
        bsem_post_all(thpool->jobqueue->has_jobs);
        time(&end);
        tpassed = difftime(end, start);
    }

    /* Poll remaining threads */
    while (thpool->num_threads_alive) {
        bsem_post_all(thpool->jobqueue->has_jobs);
        sleep(1);
    }

    /* cleanup current work groups */
    thpool_cleanup(thpool);

    /* Job queue cleanup */
    jobqueue_destroy(thpool);
    free(thpool->jobqueue);

    /* Deallocs */
    int n;
    for (n = 0; n < threads_total; n++) {
        thread_destroy(thpool->threads[n]);
    }
    free(thpool->threads);
    free(thpool);
    
    if(cppadcg_pool_verbose) {
        fprintf(stdout, "thpool_destroy(): thread pool destroyed\n");
    }
}


/* ============================ THREAD ============================== */


/* Initialize a thread in the thread pool
 *
 * @param thread        address to the pointer of the thread to be created
 * @param id            id to be given to the thread
 * @return 0 on success, -1 otherwise.
 */
static int thread_init(ThPool* thpool,
                       Thread** thread,
                       int id) {

    *thread = (Thread*) malloc(sizeof(Thread));
    if (*thread == NULL) {
        fprintf(stderr, "thread_init(): Could not allocate memory for thread\n");
        return -1;
    }

    (*thread)->thpool = thpool;
    (*thread)->id = id;
    (*thread)->processed_groups = NULL;

    pthread_create(&(*thread)->pthread, NULL, (void*) thread_do, (*thread));
    pthread_detach((*thread)->pthread);
    return 0;
}

/* What each thread is doing
*
* In principle this is an endless loop. The only time this loop gets interrupted is once
* thpool_destroy() is invoked or the program exits.
*
* @param  thread        thread that will run this function
* @return nothing
*/
static void* thread_do(Thread* thread) {
    float elapsed;
    int info;
    struct timespec cputime;
    JobQueue* queue;
    WorkGroup* workGroup;
    Job* job;
    thpool_function_type func_buff;
    void* arg_buff;
    int i;

    /* Set thread name for profiling and debugging */
    char thread_name[128] = {0};
    sprintf(thread_name, "thread-pool-%d", thread->id);

#if defined(__linux__)
    /* Use prctl instead to prevent using _GNU_SOURCE flag and implicit declaration */
    prctl(PR_SET_NAME, thread_name);
#elif defined(__APPLE__) && defined(__MACH__)
    pthread_setname_np(thread_name);
#else
    fprintf(stderr, "thread_do(): pthread_setname_np is not supported on this system");
#endif

    /* Assure all threads have been created before starting serving */
    ThPool* thpool = thread->thpool;

    /* Mark thread as alive (initialized) */
    pthread_mutex_lock(&thpool->thcount_lock);
    thpool->num_threads_alive += 1;
    pthread_mutex_unlock(&thpool->thcount_lock);

    queue = thpool->jobqueue;

    while (thpool->threads_keepalive) {

        bsem_wait(queue->has_jobs);

        if (!thpool->threads_keepalive) {
            break;
        }

        pthread_mutex_lock(&thpool->thcount_lock);
        thpool->num_threads_working++;
        pthread_mutex_unlock(&thpool->thcount_lock);

        while (thpool->threads_keepalive) {
            /* Read job from queue and execute it */
            pthread_mutex_lock(&queue->rwmutex);
            workGroup = jobqueue_pull(thpool, thread->id);
            pthread_mutex_unlock(&queue->rwmutex);

            if (workGroup == NULL)
                break;

            if (cppadcg_pool_verbose) {
                get_monotonic_time2(&workGroup->startTime);
            }

            for (i = 0; i < workGroup->size; ++i) {
                job = &workGroup->jobs[i];

                if (cppadcg_pool_verbose) {
                    get_monotonic_time2(&job->startTime);
                }

                int do_benchmark = job->elapsed != NULL;
                if (do_benchmark) {
                    elapsed = -get_thread_time(&cputime, &info);
                }

                /* Execute the job */
                func_buff = job->function;
                arg_buff = job->arg;
                func_buff(arg_buff);

                if (do_benchmark && info == 0) {
                    elapsed += get_thread_time(&cputime, &info);
                    if (info == 0) {
                        (*job->elapsed) = elapsed;
                    }
                }

                if (cppadcg_pool_verbose) {
                    get_monotonic_time2(&job->endTime);
                }
            }

            if (cppadcg_pool_verbose) {
                get_monotonic_time2(&workGroup->endTime);

                if (thread->processed_groups == NULL) {
                    thread->processed_groups = workGroup;
                } else {
                    workGroup->prev = thread->processed_groups;
                    thread->processed_groups = workGroup;
                }
            } else {
                free(workGroup->jobs);
                free(workGroup);
            }
        }

        pthread_mutex_lock(&thpool->thcount_lock);
        thpool->num_threads_working--;
        if (!thpool->num_threads_working) {
            pthread_cond_signal(&thpool->threads_all_idle);
        }
        pthread_mutex_unlock(&thpool->thcount_lock);
    }

    pthread_mutex_lock(&thpool->thcount_lock);
    thpool->num_threads_alive--;
    pthread_mutex_unlock(&thpool->thcount_lock);

    return NULL;
}


/* Frees a thread  */
static void thread_destroy(Thread* thread) {
    free(thread);
}


/* ============================ JOB QUEUE =========================== */


/* Initialize queue */
static int jobqueue_init(ThPool* thpool) {

    JobQueue* queue = (JobQueue*) malloc(sizeof(JobQueue));
    if (queue == NULL) {
        return -1;
    }
    thpool->jobqueue = queue;
    queue->len = 0;
    queue->front = NULL;
    queue->rear = NULL;
    queue->group_front = NULL;
    queue->total_time = 0;
    queue->highest_expected_return = 0;

    queue->has_jobs = (BSem*) malloc(sizeof(BSem));
    if (queue->has_jobs == NULL) {
        return -1;
    }

    pthread_mutex_init(&(queue->rwmutex), NULL);
    bsem_init(queue->has_jobs, 0);

    return 0;
}


/* Clear the queue */
static void jobqueue_clear(ThPool* thpool) {
    WorkGroup* group;
    int size;

    do {
        group = jobqueue_pull(thpool, -1);
        if (group == NULL) {
            size = 0;
        } else {
            size = group->size;
            free(group->jobs);
            free(group);
        }
    } while (size > 0);

    thpool->jobqueue->front = NULL;
    thpool->jobqueue->rear = NULL;
    bsem_reset(thpool->jobqueue->has_jobs);
    thpool->jobqueue->len = 0;
    thpool->jobqueue->group_front = NULL;
    thpool->jobqueue->total_time = 0;
    thpool->jobqueue->highest_expected_return = 0;
}


/**
 * Add (allocated) job to queue without locks (internal function)
 */
static void jobqueue_push_internal(JobQueue* queue,
                                   Job* newjob) {
    newjob->prev = NULL;

    switch (queue->len) {

        case 0:  /* if no jobs in queue */
            queue->front = newjob;
            queue->rear = newjob;
            break;

        default: /* if jobs in queue */
            queue->rear->prev = newjob;
            queue->rear = newjob;

    }
    if(newjob->avgElapsed != NULL) {
        queue->total_time += *newjob->avgElapsed;
    }
    queue->len++;
}

/**
 * Add (allocated) job to queue
 */
static void jobqueue_push(JobQueue* queue,
                          Job* newjob) {
    pthread_mutex_lock(&queue->rwmutex);

    jobqueue_push_internal(queue, newjob);

    bsem_post(queue->has_jobs);

    pthread_mutex_unlock(&queue->rwmutex);
}


/**
 * Add (allocated) multiple jobs to queue
 */
static void jobqueue_multipush(JobQueue* queue,
                               Job* newjob[],
                               int nJobs) {
    int i;

    pthread_mutex_lock(&queue->rwmutex);

    for(i = 0; i < nJobs; ++i) {
        jobqueue_push_internal(queue, newjob[i]);
    }

    bsem_post_all(queue->has_jobs);

    pthread_mutex_unlock(&queue->rwmutex);
}

static Job* jobqueue_extract_single(JobQueue* queue) {
    Job* job = queue->front;

    switch (queue->len) {
        case 0:  /* if no jobs in queue */
            return NULL;

        case 1:  /* if one job in queue */
            queue->front = NULL;
            queue->rear = NULL;
            queue->len = 0;
            queue->total_time = 0;
            queue->highest_expected_return = 0;
            return job;

        default: /* if >1 jobs in queue */
            queue->front = job->prev;
            queue->len--;
            if(job->avgElapsed != NULL) {
                queue->total_time -= *job->avgElapsed;
            }
            return job;
    }
}

static void jobqueue_extract_single_group(JobQueue* queue,
                                          WorkGroup* group) {
    Job* job = jobqueue_extract_single(queue);
    if(job != NULL) {
        group->size = 1;
        group->jobs = (Job*) malloc(sizeof(Job));
        group->jobs[0] = *job; // copy
        free(job);
    } else {
        group->size = 0;
        group->jobs = NULL;
    }
}

/**
 * Get jobs from the queue(removes them from the queue)
 *
 * Notice: Caller MUST hold a mutex
 */
static WorkGroup* jobqueue_pull(ThPool* thpool,
                                int id) {

    WorkGroup* group;
    Job* job;
    float current_time;
    float duration, duration_next, min_duration, target_duration;
    struct timespec timeAux;
    int info;
    int i;
    JobQueue* queue = thpool->jobqueue;

    if (schedule_strategy == SCHED_STATIC && queue->group_front != NULL) {
        // STATIC
        group = queue->group_front;

        queue->group_front = group->prev;
        group->prev = NULL;

    } else if (queue->len == 0) {
        // nothing to do
        group = NULL;

    } else if (schedule_strategy == SCHED_DYNAMIC || queue->len == 1 || queue->total_time <= 0) {
        // SCHED_DYNAMIC
        group = (WorkGroup*) malloc(sizeof(WorkGroup));
        group->prev = NULL;

        if (cppadcg_pool_verbose) {
            if (schedule_strategy == SCHED_GUIDED) {
                if (queue->len == 1)
                    fprintf(stdout, "jobqueue_pull(): Thread %i given a work group with 1 job\n", id);
                else if (queue->total_time <= 0)
                    fprintf(stdout, "jobqueue_pull(): Thread %i using single-job instead of multi-job (no timing information)\n", id);
            } else if (schedule_strategy == SCHED_STATIC && queue->len >= 1) {
                if (queue->total_time >= 0) {
                    // this should not happen but just in case the user messed up
                    fprintf(stderr, "jobqueue_pull(): Thread %i given a work group with 1 job\n", id);
                } else {
                    fprintf(stdout, "jobqueue_pull(): Thread %i given a work group with 1 job\n", id);
                }
            }
        }

        jobqueue_extract_single_group(thpool->jobqueue, group);
    } else { // schedule_strategy == SCHED_GUIDED
        // SCHED_GUIDED
        group = (WorkGroup*) malloc(sizeof(WorkGroup));
        group->prev = NULL;

        job = queue->front;

        if (job->avgElapsed == NULL) {
            if (cppadcg_pool_verbose) {
                fprintf(stderr, "jobqueue_pull(): Thread %i using single job instead of multi-job (No timing information for current job)\n", id);
            }
            // cannot use this strategy (something went wrong!)
            jobqueue_extract_single_group(thpool->jobqueue, group);

        } else {
            // there are at least 2 jobs in the queue
            group->size = 1;
            duration = *job->avgElapsed;
            duration_next = duration;
            job = job->prev;
            target_duration = queue->total_time * cppadcg_pool_guided_maxgroupwork / thpool->num_threads; // always positive
            current_time = get_monotonic_time(&timeAux, &info);

            if (queue->highest_expected_return > 0 && info) {
                min_duration = 0.9f * (queue->highest_expected_return - current_time);
                if (target_duration < min_duration) {
                    target_duration = min_duration;
                }
            }

            do {
                if (job->avgElapsed == NULL) {
                    break;
                }
                duration_next += *job->avgElapsed;
                if (duration_next < target_duration) {
                    group->size++;
                    duration = duration_next;
                } else {
                    break;
                }
                job = job->prev;
            } while (job != queue->front);

            if (cppadcg_pool_verbose) {
                fprintf(stdout, "jobqueue_pull(): Thread %i given a work group with %i jobs for %e s (target: %e s)\n", id, group->size, duration, target_duration);
            }

            group->jobs = (Job*) malloc(group->size * sizeof(Job));
            for (i = 0; i < group->size; ++i) {
                job = jobqueue_extract_single(thpool->jobqueue);
                group->jobs[i] = *job; // copy
                free(job);
            }

            duration_next = current_time + duration; // the time when the current work is expected to end
            if(duration_next > queue->highest_expected_return)
                queue->highest_expected_return = duration_next;
        }

    }
    /* more than one job in queue -> post it */
    if (queue->len > 0 || queue->group_front != NULL) {
        bsem_post(queue->has_jobs);
    }

    return group;
}


/* Free all queue resources back to the system */
static void jobqueue_destroy(ThPool* thpool) {
    jobqueue_clear(thpool);
    free(thpool->jobqueue->has_jobs);
}





/* ======================== SYNCHRONISATION ========================= */


/* Init semaphore to 1 or 0 */
static void bsem_init(BSem* bsem, int value) {
    if (value < 0 || value > 1) {
        fprintf(stderr, "bsem_init(): Binary semaphore can take only values 1 or 0");
        exit(1);
    }
    pthread_mutex_init(&(bsem->mutex), NULL);
    pthread_cond_init(&(bsem->cond), NULL);
    bsem->v = value;
}


/* Reset semaphore to 0 */
static void bsem_reset(BSem* bsem) {
    bsem_init(bsem, 0);
}


/* Post to at least one thread */
static void bsem_post(BSem* bsem) {
    pthread_mutex_lock(&bsem->mutex);
    bsem->v = 1;
    pthread_cond_signal(&bsem->cond);
    pthread_mutex_unlock(&bsem->mutex);
}


/* Post to all threads */
static void bsem_post_all(BSem* bsem) {
    pthread_mutex_lock(&bsem->mutex);
    bsem->v = 1;
    pthread_cond_broadcast(&bsem->cond);
    pthread_mutex_unlock(&bsem->mutex);
}


/* Wait on semaphore until semaphore has value 0 */
static void bsem_wait(BSem* bsem) {
    pthread_mutex_lock(&bsem->mutex);
    while (bsem->v != 1) {
        pthread_cond_wait(&bsem->cond, &bsem->mutex);
    }
    bsem->v = 0;
    pthread_mutex_unlock(&bsem->mutex);
}
