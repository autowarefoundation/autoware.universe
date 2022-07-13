#ifndef CPPAD_CG_MODEL_LIBRARY_C_SOURCE_GEN_IMPL_INCLUDED
#define CPPAD_CG_MODEL_LIBRARY_C_SOURCE_GEN_IMPL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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

#include <typeinfo>

namespace CppAD {
namespace cg {

template<class Base>
const unsigned long ModelLibraryCSourceGen<Base>::API_VERSION = 7;

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_VERSION = "cppad_cg_version";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_MODELS = "cppad_cg_models";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_ONCLOSE = "cppad_cg_on_close";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLDISABLED = "cppad_cg_set_thread_pool_disabled";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_ISTHREADPOOLDISABLED = "cppad_cg_is_thread_pool_disabled";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADS = "cppad_cg_set_thread_number";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADS = "cppad_cg_get_thread_number";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADSCHEDULERSTRAT = "cppad_cg_thpool_set_scheduler_strategy";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADSCHEDULERSTRAT = "cppad_cg_thpool_get_scheduler_strategy";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLVERBOSE = "cppad_cg_thpool_set_verbose";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_ISTHREADPOOLVERBOSE = "cppad_cg_thpool_is_verbose";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLGUIDEDMAXGROUPWORK = "cppad_cg_thpool_set_guided_maxgroupwork";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADPOOLGUIDEDMAXGROUPWORK = "cppad_cg_thpool_get_guided_maxgroupwork";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_SETTHREADPOOLNUMBEROFTIMEMEAS = "cppad_cg_thpool_set_number_of_time_meas";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::FUNCTION_GETTHREADPOOLNUMBEROFTIMEMEAS = "cppad_cg_thpool_get_number_of_time_meas";

template<class Base>
const std::string ModelLibraryCSourceGen<Base>::CONST = "const";

template<class Base>
void ModelLibraryCSourceGen<Base>::saveSources(const std::string& sourcesFolder) {

    // create the folder if it does not exist
    system::createFolder(sourcesFolder);

    // save/generate model sources
    for (const auto& it : _models) {
        saveSources(sourcesFolder, it.second->getSources());
    }

    // save/generate library sources
    saveSources(sourcesFolder, getLibrarySources());

    // save custom user sources
    saveSources(sourcesFolder, getCustomSources());
}

template<class Base>
void ModelLibraryCSourceGen<Base>::saveSources(const std::string& sourcesFolder,
                                               const std::map<std::string, std::string>& sources) {
    for (const auto& it : sources) {
        // for debugging purposes only
        std::ofstream sourceFile;
        std::string file = system::createPath(sourcesFolder, it.first);
        sourceFile.open(file.c_str());
        sourceFile << it.second;
        sourceFile.close();
    }
}

template<class Base>
const std::map<std::string, std::string>& ModelLibraryCSourceGen<Base>::getLibrarySources() {
    if (_libSources.empty()) {
        generateVersionSource(_libSources);
        generateModelsSource(_libSources);
        generateOnCloseSource(_libSources);
        generateThreadPoolSources(_libSources);

        if(_multiThreading != MultiThreadingType::NONE) {
            bool usingMultiThreading = false;
            for (const auto& it : _models) {
                if (it.second->isJacobianMultiThreadingEnabled() || it.second->isHessianMultiThreadingEnabled()) {
                    usingMultiThreading = true;
                    break;
                }
            }

            if (usingMultiThreading) {
                if (_multiThreading == MultiThreadingType::PTHREADS) {
                    _libSources["thread_pool.c"] = CPPADCG_PTHREAD_POOL_C_FILE;

                } else if (_multiThreading == MultiThreadingType::OPENMP) {
                    _libSources["thread_pool.c"] = CPPADCG_OPENMP_C_FILE;
                }
            }
        }
    }

    return _libSources;
}

template<class Base>
void ModelLibraryCSourceGen<Base>::generateVersionSource(std::map<std::string, std::string>& sources) {
    _cache.str("");
    _cache << "unsigned long " << FUNCTION_VERSION << "() {\n"
            << "   return " << API_VERSION << "u;\n"
            << "}\n\n";

    sources[FUNCTION_VERSION + ".c"] = _cache.str();
}

template<class Base>
void ModelLibraryCSourceGen<Base>::generateModelsSource(std::map<std::string, std::string>& sources) {
    _cache.str("");
    LanguageC<Base>::printFunctionDeclaration(_cache, "void", FUNCTION_MODELS, {"char const *const** names",
                                                                                "int* count"});
    _cache << " {\n"
            "   static const char* const models[] = {\n";

    for (auto it = _models.begin(); it != _models.end(); ++it) {
        if (it != _models.begin()) {
            _cache << ",\n";
        }
        _cache << "      \"" << it->first << "\"";
    }
    _cache << "};\n"
            "   *names = models;\n"
            "   *count = " << _models.size() << ";\n"
            "}\n\n";

    sources[FUNCTION_MODELS + ".c"] = _cache.str();
}

template<class Base>
void ModelLibraryCSourceGen<Base>::generateOnCloseSource(std::map<std::string, std::string>& sources) {
    bool pthreads = false;
    if(_multiThreading == MultiThreadingType::PTHREADS) {
        for (const auto& it : _models) {
            if (it.second->isJacobianMultiThreadingEnabled() || it.second->isHessianMultiThreadingEnabled()) {
                pthreads = true;
                break;
            }
        }
    }

    _cache.str("");
    if (pthreads) {
        _cache << CPPADCG_PTHREAD_POOL_H_FILE << "\n\n";
    }
    _cache << "void " << FUNCTION_ONCLOSE << "() {\n";
    if (pthreads) {
        _cache << "cppadcg_thpool_shutdown();\n";
    }
    _cache << "}\n\n";

    sources[FUNCTION_ONCLOSE + ".c"] = _cache.str();
}

template<class Base>
void ModelLibraryCSourceGen<Base>::generateThreadPoolSources(std::map<std::string, std::string>& sources) {

    bool usingMultiThreading = false;
    if(_multiThreading != MultiThreadingType::NONE) {
        for (const auto& it : _models) {
            if (it.second->isJacobianMultiThreadingEnabled() || it.second->isHessianMultiThreadingEnabled()) {
                usingMultiThreading = true;
                break;
            }
        }
    }

    if (usingMultiThreading && _multiThreading == MultiThreadingType::PTHREADS) {
        _cache.str("");
        _cache << CPPADCG_PTHREAD_POOL_H_FILE << "\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLDISABLED << "(int disabled) {\n";
        _cache << "   cppadcg_thpool_set_disabled(disabled);\n";
        _cache << "}\n\n";

        _cache << "int " << FUNCTION_ISTHREADPOOLDISABLED << "() {\n";
        _cache << "   return cppadcg_thpool_is_disabled();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADS << "(unsigned int n) {\n";
        _cache << "   cppadcg_thpool_set_threads(n);\n";
        _cache << "}\n\n";

        _cache << "unsigned int " << FUNCTION_GETTHREADS << "() {\n";
        _cache << "   return cppadcg_thpool_get_threads();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADSCHEDULERSTRAT << "(enum ScheduleStrategy s) {\n";
        _cache << "   cppadcg_thpool_set_scheduler_strategy(s);\n";
        _cache << "}\n\n";

        _cache << "enum ScheduleStrategy " << FUNCTION_GETTHREADSCHEDULERSTRAT << "() {\n";
        _cache << "   return cppadcg_thpool_get_scheduler_strategy();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLVERBOSE << "(int v) {\n";
        _cache << "   cppadcg_thpool_set_verbose(v);\n";
        _cache << "}\n\n";

        _cache << "int " << FUNCTION_ISTHREADPOOLVERBOSE << "() {\n";
        _cache << "   return cppadcg_thpool_is_verbose();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLGUIDEDMAXGROUPWORK << "(float v) {\n";
        _cache << "   cppadcg_thpool_set_guided_maxgroupwork(v);\n";
        _cache << "}\n\n";

        _cache << "float " << FUNCTION_GETTHREADPOOLGUIDEDMAXGROUPWORK << "() {\n";
        _cache << "   return cppadcg_thpool_get_guided_maxgroupwork();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLNUMBEROFTIMEMEAS << "(unsigned int n) {\n";
        _cache << "   cppadcg_thpool_set_n_time_meas(n);\n";
        _cache << "}\n\n";

        _cache << "unsigned int " << FUNCTION_GETTHREADPOOLNUMBEROFTIMEMEAS << "() {\n";
        _cache << "   return cppadcg_thpool_get_n_time_meas();\n";
        _cache << "}\n\n";

        sources["thread_pool_access.c"] = _cache.str();

    } else if(usingMultiThreading && _multiThreading == MultiThreadingType::OPENMP) {
        _cache.str("");
        _cache << "#include <omp.h>\n";
        _cache << CPPADCG_OPENMP_H_FILE << "\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLDISABLED << "(int disabled) {\n";
        _cache << "   cppadcg_openmp_set_disabled(disabled);\n";
        _cache << "}\n\n";

        _cache << "int " << FUNCTION_ISTHREADPOOLDISABLED << "() {\n";
        _cache << "   return cppadcg_openmp_is_disabled();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADS << "(unsigned int n) {\n";
        _cache << "   cppadcg_openmp_set_threads(n);\n";
        _cache << "}\n\n";

        _cache << "unsigned int " << FUNCTION_GETTHREADS << "() {\n";
        _cache << "   return cppadcg_openmp_get_threads();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADSCHEDULERSTRAT << "(enum ScheduleStrategy s) {\n";
        _cache << "   cppadcg_openmp_set_scheduler_strategy(s);\n";
        _cache << "}\n\n";

        _cache << "enum ScheduleStrategy " << FUNCTION_GETTHREADSCHEDULERSTRAT << "() {\n";
        _cache << "   return cppadcg_openmp_get_scheduler_strategy();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLVERBOSE << "(int v) {\n";
        _cache << "   cppadcg_openmp_set_verbose(v);\n";
        _cache << "}\n\n";

        _cache << "int " << FUNCTION_ISTHREADPOOLVERBOSE << "() {\n";
        _cache << "   return cppadcg_openmp_is_verbose();\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLGUIDEDMAXGROUPWORK << "(float v) {\n";
        _cache << "}\n\n";

        _cache << "float " << FUNCTION_GETTHREADPOOLGUIDEDMAXGROUPWORK << "() {\n";
        _cache << "   return 1.0;\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLNUMBEROFTIMEMEAS << "(unsigned int n) {\n";
        _cache << "}\n\n";

        _cache << "unsigned int " << FUNCTION_GETTHREADPOOLNUMBEROFTIMEMEAS << "() {\n";
        _cache << "   return 0;\n";
        _cache << "}\n\n";

        sources["thread_pool_access.c"] = _cache.str();

    } else {
        _cache.str("");
        _cache << "enum ScheduleStrategy {SCHED_STATIC = 1, SCHED_DYNAMIC = 2, SCHED_GUIDED = 3};\n"
                "\n";
        _cache << "void " << FUNCTION_SETTHREADPOOLDISABLED << "(int disabled) {\n";
        _cache << "}\n\n";

        _cache << "int " << FUNCTION_ISTHREADPOOLDISABLED << "() {\n";
        _cache << "   return 1;\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADS << "(unsigned int n) {\n";
        _cache << "}\n\n";

        _cache << "unsigned int " << FUNCTION_GETTHREADS << "() {\n";
        _cache << "   return 1;\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADSCHEDULERSTRAT << "(enum ScheduleStrategy s) {\n";
        _cache << "}\n\n";

        _cache << "enum ScheduleStrategy " << FUNCTION_GETTHREADSCHEDULERSTRAT << "() {\n";
        _cache << "   return SCHED_STATIC;\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLVERBOSE << "(int v) {\n";
        _cache << "}\n\n";

        _cache << "int " << FUNCTION_ISTHREADPOOLVERBOSE << "() {\n";
        _cache << "   return 0;\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLGUIDEDMAXGROUPWORK << "(float v) {\n";
        _cache << "}\n\n";

        _cache << "float " << FUNCTION_GETTHREADPOOLGUIDEDMAXGROUPWORK << "() {\n";
        _cache << "   return 1.0;\n";
        _cache << "}\n\n";

        _cache << "void " << FUNCTION_SETTHREADPOOLNUMBEROFTIMEMEAS << "(unsigned int n) {\n";
        _cache << "}\n\n";

        _cache << "unsigned int " << FUNCTION_GETTHREADPOOLNUMBEROFTIMEMEAS << "() {\n";
        _cache << "   return 0;\n";
        _cache << "}\n\n";

        sources["thread_pool_access.c"] = _cache.str();
    }
}

} // END cg namespace
} // END CppAD namespace

#endif