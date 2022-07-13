#ifndef CPPAD_CG_JOB_TIMER_INCLUDED
#define CPPAD_CG_JOB_TIMER_INCLUDED
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

namespace CppAD {
namespace cg {

/**
 * A type of executing task
 */
class JobType {
private:
    /**
     * action name for the beginning of the task
     */
    std::string _action;
    /**
     * action name for the completion of the task
     */
    std::string _actionEnd;
public:

    inline JobType(const std::string& action,
                   const std::string& actionEnd) :
        _action(action),
        _actionEnd(actionEnd) {
    }

    inline const std::string& getActionName() const {
        return _action;
    }

    inline void setActionName(const std::string& action) {
        _action = action;
    }

    inline const std::string& getActionEndName()const {
        return _actionEnd;
    }

    inline void setActionEndName(const std::string& actionEnd) {
        _actionEnd = actionEnd;
    }

    inline virtual ~JobType() {
    }
};

/**
 * Holds several job types
 */
template<int T = 0 >
class JobTypeHolder {
public:
    static const JobType DEFAULT;
    static const JobType LOOP_DETECTION;
    static const JobType GRAPH;
    static const JobType SOURCE_FOR_MODEL;
    static const JobType SOURCE_GENERATION;
    static const JobType COMPILING_FOR_MODEL;
    static const JobType COMPILING;
    static const JobType COMPILING_DYNAMIC_LIBRARY;
    static const JobType DYNAMIC_MODEL_LIBRARY;
    static const JobType STATIC_MODEL_LIBRARY;
    static const JobType ASSEMBLE_STATIC_LIBRARY;
    static const JobType JIT_MODEL_LIBRARY;
};

template<int T>
const JobType JobTypeHolder<T>::DEFAULT("generating", "generated");

template<int T>
const JobType JobTypeHolder<T>::LOOP_DETECTION("starting loop detection", "ended loop detection");

template<int T>
const JobType JobTypeHolder<T>::GRAPH("creating operation graph for", "created operation graph for");

template<int T>
const JobType JobTypeHolder<T>::SOURCE_FOR_MODEL("source-code for model", "source-code for model");

template<int T>
const JobType JobTypeHolder<T>::SOURCE_GENERATION("generating source for", "generated source for");

template<int T>
const JobType JobTypeHolder<T>::COMPILING_FOR_MODEL("compiling object files", "compiled object files");

template<int T>
const JobType JobTypeHolder<T>::COMPILING("compiling", "compiled");

template<int T>
const JobType JobTypeHolder<T>::COMPILING_DYNAMIC_LIBRARY("compiling dynamic library", "compiled library");

template<int T>
const JobType JobTypeHolder<T>::DYNAMIC_MODEL_LIBRARY("creating library", "created library");

template<int T>
const JobType JobTypeHolder<T>::STATIC_MODEL_LIBRARY("creating library", "created library");

template<int T>
const JobType JobTypeHolder<T>::ASSEMBLE_STATIC_LIBRARY("assembling static library", "assembled static library");

template<int T>
const JobType JobTypeHolder<T>::JIT_MODEL_LIBRARY("preparing JIT library", "prepared JIT library");

/**
 * Represents a task for which the execution time will be determined
 */
class Job {
private:
    /**
     * type
     */
    const JobType* _type;
    /**
     * Job name
     */
    std::string _name;
    /**
     * Job starting time
     */
    std::chrono::steady_clock::time_point _beginTime;
    /**
     * Whether or not there are/were other jobs inside
     */
    bool _nestedJobs;
public:

    inline Job(const JobType& type,
               const std::string& name) :
        _type(&type),
        _name(name),
        _beginTime(std::chrono::steady_clock::now()),
        _nestedJobs(false) {
    }

    inline const JobType& getType()const {
        return *_type;
    }

    inline const std::string& name() const {
        return _name;
    }

    inline std::chrono::steady_clock::time_point beginTime() const {
        return _beginTime;
    }

    inline virtual ~Job() {
    }

    friend class JobTimer;
};

/**
 * A listener for job start/end events
 */
class JobListener {
public:
    using duration = std::chrono::steady_clock::duration;

    virtual void jobStarted(const std::vector<Job>& job) = 0;

    virtual void jobEndended(const std::vector<Job>& job,
                             duration elapsed) = 0;
};

/**
 * Utility class used to print elapsed times of jobs
 */
class JobTimer : public JobTypeHolder<> {
protected:
    /**
     * Whether or not to print progress information to the standard
     * output
     */
    bool _verbose;
private:
    /**
     * saves the current job names
     */
    std::vector<Job> _jobs;
    /**
     *
     */
    size_t _maxLineWidth;
    /**
     * number of spaces per indentation level
     */
    size_t _indent;
    /**
     *
     */
    std::ostringstream _os;
    /**
     *
     */
    std::string _action;
    /**
     *
     */
    std::string _actionEnd;
    /**
     *
     */
    std::set<JobListener*> _listeners;
public:

    JobTimer() :
        _verbose(false),
        _maxLineWidth(80),
        _indent(2) {
    }

    inline bool isVerbose() const {
        return _verbose;
    }

    inline void setVerbose(bool verbose) {
        _verbose = verbose;
    }

    inline size_t getMaxLineWidth() const {
        return _maxLineWidth;
    }

    inline void setMaxLineWidth(size_t width) {
        _maxLineWidth = width;
    }

    /**
     * Provides the number of currently running jobs
     *
     * @return the number of running jobs
     */
    inline size_t getJobCount() const {
        return _jobs.size();
    }

    inline void addListener(JobListener& l) {
        _listeners.insert(&l);
    }

    inline bool removeListener(JobListener& l) {
        return _listeners.erase(&l) > 0;
    }

    inline void startingJob(const std::string& jobName,
                            const JobType& type = JobTypeHolder<>::DEFAULT,
                            const std::string& prefix = "") {

        _jobs.push_back(Job(type, jobName));

        if (_verbose) {
            OStreamConfigRestore osr(std::cout);

            Job& job = _jobs.back();

            size_t indent = 0;
            if (_jobs.size() > 1) {
                Job& parent = _jobs[_jobs.size() - 2]; // must be after adding job
                if (!parent._nestedJobs) {
                    parent._nestedJobs = true;
                    std::cout << "\n";
                }
                indent = _indent * (_jobs.size() - 1);
            }

            _os.str("");
            if (indent > 0) _os << std::string(indent, ' ');
            if (!prefix.empty()) _os << prefix << " ";
            _os << type.getActionName() << " " << job.name() << " ...";

            char f = std::cout.fill();
            std::cout << std::setw(_maxLineWidth) << std::setfill('.') << std::left << _os.str();
            std::cout.flush();
            std::cout.fill(f); // restore fill character
        }

        // notify listeners
        for (JobListener* l : _listeners) {
            l->jobStarted(_jobs);
        }
    }

    inline void finishedJob() {
        using namespace std::chrono;

        CPPADCG_ASSERT_UNKNOWN(_jobs.size() > 0);

        Job& job = _jobs.back();

        std::chrono::steady_clock::duration elapsed = steady_clock::now() - job.beginTime();

        if (_verbose) {
            OStreamConfigRestore osr(std::cout);

            if (job._nestedJobs) {
                _os.str("");
                if (!_jobs.empty())
                    _os << std::string(_indent * (_jobs.size() - 1), ' ');
                _os << job.getType().getActionEndName() << " " << job.name() << " ...";

                char f = std::cout.fill();
                std::cout << std::setw(_maxLineWidth) << std::setfill('.') << std::left << _os.str();
                std::cout.fill(f); // restore fill character
            }

            std::cout << " done [" << std::fixed << std::setprecision(3) << duration<float>(elapsed).count() << "]" << std::endl;
        }

        // notify listeners
        for (JobListener* l : _listeners) {
            l->jobEndended(_jobs, elapsed);
        }

        _jobs.pop_back();
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
