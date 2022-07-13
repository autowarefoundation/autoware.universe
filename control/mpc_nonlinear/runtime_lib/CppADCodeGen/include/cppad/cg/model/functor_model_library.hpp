#ifndef CPPAD_CG_FUNCTOR_MODEL_LIBRARY_INCLUDED
#define CPPAD_CG_FUNCTOR_MODEL_LIBRARY_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

namespace CppAD {
namespace cg {

/**
 * Abstract class used to load models
 *
 * @author Joao Leal
 */
template<class Base>
class FunctorModelLibrary : public ModelLibrary<Base> {
protected:
    std::set<std::string> _modelNames;
    unsigned long _version; // API version
    void (*_onClose)();
    void (*_setThreadPoolDisabled)(int);
    int (*_isThreadPoolDisabled)();
    void (*_setThreads)(unsigned int);
    unsigned int (*_getThreads)();
    void (*_setSchedulerStrategy)(int);
    int (*_getSchedulerStrategy)();
    void (*_setThreadPoolVerbose)(int v);
    int (*_isThreadPoolVerbose)();
    void (*_setThreadPoolGuidedMaxWork)(float v);
    float (*_getThreadPoolGuidedMaxWork)();
    void (*_setThreadPoolNumberOfTimeMeas)(unsigned int n);
    unsigned int (*_getThreadPoolNumberOfTimeMeas)();
public:

    std::set<std::string> getModelNames() override {
        return _modelNames;
    }

    /**
     * Creates a new FunctorGenericModel object that can be used to evaluate
     * the model.
     *
     * @param modelName The model name.
     * @return The model object or nullptr if no model exists with the provided
     *         name.
     */
    virtual std::unique_ptr<FunctorGenericModel<Base>> modelFunctor(const std::string& modelName) = 0;

    std::unique_ptr<GenericModel<Base>> model(const std::string& modelName) override final {
        return std::unique_ptr<GenericModel<Base>> (modelFunctor(modelName).release());
    }

    /**
     * Provides the API version used to create the model library.
     *
     * @return the API version
     */
    virtual unsigned long getAPIVersion() {
        return _version;
    }

    /**
     * Provides a pointer to a function in the model library.
     *
     * @param functionName The name of the function in the dynamic library
     * @param required Whether or not the function symbol must exist in the
     *                 library. If the function is required and does not
     *                 exist then the CppAD error handler is called, if it
     *                 is not required and it does not exist then nullptr is
     *                 return.
     * @return A pointer to the function symbol in the dynamic library if it
     *         exists, nullptr otherwise.
     * @throws CGException If there is a problem loading the function symbol
     */
    virtual void* loadFunction(const std::string& functionName,
                               bool required = true) = 0;

    void setThreadPoolDisabled(bool disabled) override {
        if(_setThreadPoolDisabled != nullptr) {
            (*_setThreadPoolDisabled)(disabled);
        }
    }

    bool isThreadPoolDisabled() const override {
        if(_isThreadPoolDisabled != nullptr) {
            return bool((*_isThreadPoolDisabled)());
        }
        return true;
    }

    unsigned int getThreadNumber() const override {
        if (_getThreads != nullptr) {
            return (*_getThreads)();
        }
        return 1;
    }

    void setThreadNumber(unsigned int n) override {
        if (_setThreads != nullptr) {
            (*_setThreads)(n);
        }
    }

    ThreadPoolScheduleStrategy getThreadPoolSchedulerStrategy() const override {
        if (_getSchedulerStrategy != nullptr) {
            return ThreadPoolScheduleStrategy((*_getSchedulerStrategy)());
        }
        return ThreadPoolScheduleStrategy::DYNAMIC;
    }

    void setThreadPoolSchedulerStrategy(ThreadPoolScheduleStrategy s) override {
        if (_setSchedulerStrategy != nullptr) {
            (*_setSchedulerStrategy)(int(s));
        }
    }

    void setThreadPoolVerbose(bool v) override {
        if (_setThreadPoolVerbose != nullptr) {
            (*_setThreadPoolVerbose)(int(v));
        }
    }

    bool isThreadPoolVerbose() const override {
        if (_isThreadPoolVerbose != nullptr) {
            return bool((*_isThreadPoolVerbose)());
        }
        return false;
    }

    void setThreadPoolGuidedMaxWork(float v) override {
        if (_setThreadPoolGuidedMaxWork != nullptr) {
            (*_setThreadPoolGuidedMaxWork)(v);
        }
    }

    float getThreadPoolGuidedMaxWork() const override {
        if (_getThreadPoolGuidedMaxWork != nullptr) {
            return (*_getThreadPoolGuidedMaxWork)();
        }
        return 1.0;
    }

    void setThreadPoolNumberOfTimeMeas(unsigned int n) override {
        if (_setThreadPoolNumberOfTimeMeas != nullptr) {
            (*_setThreadPoolNumberOfTimeMeas)(n);
        }
    }

    unsigned int getThreadPoolNumberOfTimeMeas() const override {
        if (_getThreadPoolNumberOfTimeMeas != nullptr) {
            return (*_getThreadPoolNumberOfTimeMeas)();
        }
        return 0;
    }

    inline virtual ~FunctorModelLibrary() = default;

protected:
    FunctorModelLibrary() :
            _version(0), // not really required (but it avoids warnings)
            _onClose(nullptr),
            _setThreadPoolDisabled(nullptr),
            _isThreadPoolDisabled(nullptr),
            _setThreads(nullptr),
            _getThreads(nullptr),
            _setSchedulerStrategy(nullptr),
            _getSchedulerStrategy(nullptr),
            _setThreadPoolVerbose(nullptr),
            _isThreadPoolVerbose(nullptr),
            _setThreadPoolGuidedMaxWork(nullptr),
            _getThreadPoolGuidedMaxWork(nullptr),
            _setThreadPoolNumberOfTimeMeas(nullptr),
            _getThreadPoolNumberOfTimeMeas(nullptr) {
    }

    inline void validate() {
        /**
         * Check the version
         */
        unsigned long (*versionFunc)();
        versionFunc = reinterpret_cast<decltype(versionFunc)> (loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_VERSION));

        _version = (*versionFunc)();
        if (ModelLibraryCSourceGen<Base>::API_VERSION != _version)
            throw CGException("The API version of the dynamic library (", _version,
                              ") is incompatible with the current version (",
                              ModelLibraryCSourceGen<Base>::API_VERSION, ")");

        /**
         * Load the list of models
         */
        void (*modelsFunc)(char const *const**, int*);
        modelsFunc = reinterpret_cast<decltype(modelsFunc)> (loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_MODELS));

        char const*const* model_names = nullptr;
        int model_count;
        (*modelsFunc)(&model_names, &model_count);

        for (int i = 0; i < model_count; i++) {
            _modelNames.insert(model_names[i]);
        }

        /**
         * Load the the on close function
         */
        _onClose = reinterpret_cast<decltype(_onClose)> (loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_ONCLOSE, false));

        /**
         * Thread pool related functions
         */
        _setThreadPoolDisabled = reinterpret_cast<decltype(_setThreadPoolDisabled)> (loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLDISABLED, false));
        _isThreadPoolDisabled = reinterpret_cast<decltype(_isThreadPoolDisabled)> (loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_ISTHREADPOOLDISABLED, false));
        _setThreads = reinterpret_cast<decltype(_setThreads)> (loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADS, false));
        _getThreads = reinterpret_cast<decltype(_getThreads)> (loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADS, false));
        _setSchedulerStrategy = reinterpret_cast<decltype(_setSchedulerStrategy)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADSCHEDULERSTRAT, false));
        _getSchedulerStrategy = reinterpret_cast<decltype(_getSchedulerStrategy)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADSCHEDULERSTRAT, false));
        _setThreadPoolVerbose = reinterpret_cast<decltype(_setThreadPoolVerbose)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLVERBOSE, false));
        _isThreadPoolVerbose = reinterpret_cast<decltype(_isThreadPoolVerbose)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_ISTHREADPOOLVERBOSE, false));
        _setThreadPoolGuidedMaxWork = reinterpret_cast<decltype(_setThreadPoolGuidedMaxWork)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLGUIDEDMAXGROUPWORK, false));
        _getThreadPoolGuidedMaxWork = reinterpret_cast<decltype(_getThreadPoolGuidedMaxWork)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADPOOLGUIDEDMAXGROUPWORK, false));
        _setThreadPoolNumberOfTimeMeas = reinterpret_cast<decltype(_setThreadPoolNumberOfTimeMeas)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLNUMBEROFTIMEMEAS, false));
        _getThreadPoolNumberOfTimeMeas = reinterpret_cast<decltype(_getThreadPoolNumberOfTimeMeas)> (this->loadFunction(ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADPOOLNUMBEROFTIMEMEAS, false));

        if(_setThreads != nullptr) {
            (*_setThreads)(std::thread::hardware_concurrency());
        }
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
