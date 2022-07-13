#ifndef CPPAD_CG_LLVM_BASE_MODEL_LIBRARY_PROCESSOR_INCLUDED
#define CPPAD_CG_LLVM_BASE_MODEL_LIBRARY_PROCESSOR_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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
 * Base class for the creation of a model libraries using LLVM.
 *
 * @author Joao Leal
 */
template<class Base>
class LlvmBaseModelLibraryProcessor : public ModelLibraryProcessor<Base> {
public:

    inline explicit LlvmBaseModelLibraryProcessor(ModelLibraryCSourceGen<Base>& modelLibraryHelper) :
            ModelLibraryProcessor<Base>(modelLibraryHelper) {
    }

    inline virtual ~LlvmBaseModelLibraryProcessor() = default;

protected:

    static std::string findInternalClangCHeaders(const std::string& version,
                                                 const std::string& resourceDir) {
        // check existing paths
        for (const std::string& path : explode(resourceDir, " ")) {
            if (system::isFile(system::createPath(path, system::createPath("include", "stddef.h")))) {
                return ""; // no need to add anything
            }
        }

#ifdef CPPAD_CG_SYSTEM_LINUX
        std::string clangHeaders = "/usr/lib/clang/" + version + "/include";
        if (system::isDirectory(clangHeaders)) {
            return clangHeaders; // found them
        }
#endif

        // failed to locate headers (hope they are not needed...)
        return "";
    }

    const std::set<std::string>& createBitCode(ClangCompiler<Base>& clang,
                                               const std::string& version) {
        // backup output format so that it can be restored
        OStreamConfigRestore coutb(std::cout);

        if (clang.getVersion() != version) {
            auto expected = ClangCompiler<Base>::parseVersion(version);
            auto execVersion = ClangCompiler<Base>::parseVersion(clang.getVersion());
            bool error = expected.size() > execVersion.size();
            if (!error) {
                for (size_t i = 0; i < expected.size(); ++i) {
                    if (expected[i] != execVersion[i]) {
                        error = true;
                        break;
                    }
                }
            }
            if (error) {
                throw CGException("Expected a clang with version '", version, "' but found version '", clang.getVersion(), "'");
            }
        }

        const std::map<std::string, ModelCSourceGen<Base>*>& models = this->modelLibraryHelper_->getModels();
        try {
            /**
             * generate bit code
             */
            for (const auto& p : models) {
                const std::map<std::string, std::string>& modelSources = this->getSources(*p.second);

                this->modelLibraryHelper_->startingJob("", JobTimer::COMPILING_FOR_MODEL);
                clang.generateLLVMBitCode(modelSources, this->modelLibraryHelper_);
                this->modelLibraryHelper_->finishedJob();
            }

            const std::map<std::string, std::string>& sources = this->getLibrarySources();
            clang.generateLLVMBitCode(sources, this->modelLibraryHelper_);

            const std::map<std::string, std::string>& customSource = this->modelLibraryHelper_->getCustomSources();
            clang.generateLLVMBitCode(customSource, this->modelLibraryHelper_);
        } catch (...) {
            clang.cleanup();
            throw;
        }

        return clang.getBitCodeFiles();
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
