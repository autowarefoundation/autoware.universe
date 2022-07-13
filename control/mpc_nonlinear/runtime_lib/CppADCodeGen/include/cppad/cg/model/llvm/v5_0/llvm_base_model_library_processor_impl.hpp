#ifndef CPPAD_CG_LLVM_BASE_MODEL_LIBRARY_PROCESSOR_IMPL_INCLUDED
#define CPPAD_CG_LLVM_BASE_MODEL_LIBRARY_PROCESSOR_IMPL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2017 Ciengis
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

#include <cppad/cg/model/llvm/llvm_base_model_library_processor.hpp>

namespace CppAD {
namespace cg {

/**
 * Useful class for generating a JIT evaluated model library (LLVM 5.0, 6.0, 7.0, 8.0).
 *
 * @author Joao Leal
 */
template<class Base>
class LlvmBaseModelLibraryProcessorImpl : public LlvmBaseModelLibraryProcessor<Base> {
protected:
    const std::string _version;
    std::vector<std::string> _includePaths;
    std::shared_ptr<llvm::LLVMContext> _context; // must be deleted after _linker and _module (it must come first)
    std::unique_ptr<llvm::Linker> _linker;
    std::unique_ptr<llvm::Module> _module;
public:

    /**
     * Creates a LLVM model library processor.
     *
     * @param librarySourceGen
     */
    LlvmBaseModelLibraryProcessorImpl(ModelLibraryCSourceGen<Base>& librarySourceGen,
                                      std::string version) :
        LlvmBaseModelLibraryProcessor<Base>(librarySourceGen),
            _version(std::move(version)) {
    }

    virtual ~LlvmBaseModelLibraryProcessorImpl() = default;

    /**
     * @return The version of LLVM (and Clang).
     */
    inline const std::string& getVersion() const {
        return _version;
    }

    /**
     * Define additional header paths.
     */
    inline void setIncludePaths(const std::vector<std::string>& includePaths) {
        _includePaths = includePaths;
    }

    /**
     * User defined header paths.
     */
    inline const std::vector<std::string>& getIncludePaths() const {
        return _includePaths;
    }

    /**
     *
     * @return a model library
     */
    std::unique_ptr<LlvmModelLibrary<Base>> create() {
        // backup output format so that it can be restored
        OStreamConfigRestore coutb(std::cout);

        _linker.reset(nullptr);

        this->modelLibraryHelper_->startingJob("", JobTimer::JIT_MODEL_LIBRARY);

        llvm::InitializeAllTargetMCs();
        llvm::InitializeAllTargets();
        llvm::InitializeAllAsmPrinters();

        _context.reset(new llvm::LLVMContext());

        const std::map<std::string, ModelCSourceGen<Base>*>& models = this->modelLibraryHelper_->getModels();
        for (const auto& p : models) {
            const std::map<std::string, std::string>& modelSources = this->getSources(*p.second);
            createLlvmModules(modelSources);
        }

        const std::map<std::string, std::string>& sources = this->getLibrarySources();
        createLlvmModules(sources);

        const std::map<std::string, std::string>& customSource = this->modelLibraryHelper_->getCustomSources();
        createLlvmModules(customSource);

        llvm::InitializeNativeTarget();

        std::unique_ptr<LlvmModelLibrary<Base>> lib(new LlvmModelLibraryImpl<Base>(std::move(_module), _context));

        this->modelLibraryHelper_->finishedJob();

        return lib;
    }

    /**
     * Creates a LLVM model library using an external Clang compiler to
     * generate the bitcode.
     *
     * @param clang  the external compiler
     * @return  a model library
     */
    std::unique_ptr<LlvmModelLibrary<Base>> create(ClangCompiler<Base>& clang) {
        using namespace llvm;

        // backup output format so that it can be restored
        OStreamConfigRestore coutb(std::cout);

        _linker.release();

        std::unique_ptr<LlvmModelLibrary<Base>> lib;

        this->modelLibraryHelper_->startingJob("", JobTimer::JIT_MODEL_LIBRARY);

        try {
            /**
             * generate bit code
             */
            const std::set<std::string>& bcFiles = this->createBitCode(clang, _version);

            /**
             * Load bit code and create a single module
             */
            llvm::InitializeAllTargets();
            llvm::InitializeAllAsmPrinters();

            _context.reset(new llvm::LLVMContext());

            std::unique_ptr<Module> linkerModule;

            for (const std::string& itbc : bcFiles) {
                // load bitcode file

                ErrorOr<std::unique_ptr<MemoryBuffer>> buffer = MemoryBuffer::getFile(itbc);
                if (buffer.get() == nullptr) {
                    throw CGException(buffer.getError().message());
                }

                // create the module
                Expected<std::unique_ptr<Module>> moduleOrError = llvm::parseBitcodeFile(buffer.get()->getMemBufferRef(), *_context.get());
                if (!moduleOrError) {
                    std::ostringstream error;
                    size_t nError = 0;
                    handleAllErrors(moduleOrError.takeError(), [&](ErrorInfoBase& eib) {
                        if (nError > 0) error << "; ";
                        error << eib.message();
                        nError++;
                    });
                    throw CGException(error.str());
                }

                // link modules together
                if (_linker.get() == nullptr) {
                    linkerModule = std::move(moduleOrError.get());
                    _linker.reset(new llvm::Linker(*linkerModule)); // module not destroyed
                } else {
                    if (_linker->linkInModule(std::move(moduleOrError.get()))) { // module destroyed
                        throw CGException("Failed to link");
                    }
                }
            }

            llvm::InitializeNativeTarget();

            // voila
            lib.reset(new LlvmModelLibraryImpl<Base>(std::move(linkerModule), _context));

        } catch (...) {
            clang.cleanup();
            throw;
        }
        clang.cleanup();

        this->modelLibraryHelper_->finishedJob();

        return lib;
    }

protected:

    virtual void createLlvmModules(const std::map<std::string, std::string>& sources) {
        for (const auto& p : sources) {
            createLlvmModule(p.first, p.second);
        }
    }

    virtual void createLlvmModule(const std::string& filename,
                                  const std::string& source) {
        using namespace llvm;
        using namespace clang;

        ArrayRef<StringRef> paths;
        llvm::sys::findProgramByName("clang", paths);

        IntrusiveRefCntPtr<DiagnosticOptions> diagOpts = new DiagnosticOptions();
        auto* diagClient = new TextDiagnosticPrinter(llvm::errs(), &*diagOpts); // will be owned by diags
        IntrusiveRefCntPtr<DiagnosticIDs> diagID(new DiagnosticIDs());
        IntrusiveRefCntPtr<DiagnosticsEngine> diags(new DiagnosticsEngine(diagID, &*diagOpts, diagClient));

        ArrayRef<const char*> args {"-Wall", "-x", "c", "string-input"}; // -Wall or -v flag is required to avoid an error inside createInvocationFromCommandLine()
        std::shared_ptr<CompilerInvocation> invocation(createInvocationFromCommandLine(args, diags));
        if (invocation == nullptr)
            throw CGException("Failed to create compiler invocation");

        //invocation->TargetOpts->Triple = llvm::sys::getDefaultTargetTriple();

        CompilerInvocation::setLangDefaults(*invocation->getLangOpts(), InputKind::C,
                                            llvm::Triple(invocation->TargetOpts->Triple),
                                            invocation->getPreprocessorOpts(),
                                            LangStandard::lang_unspecified);
        invocation->getFrontendOpts().DisableFree = false; // make sure we free memory (by default it does not)

        // Create a compiler instance to handle the actual work.
        CompilerInstance compiler;
        compiler.setInvocation(invocation);


        // Create the compilers actual diagnostics engine.
        compiler.createDiagnostics(); //compiler.createDiagnostics(argc, const_cast<char**> (argv));
        if (!compiler.hasDiagnostics())
            throw CGException("No diagnostics");

        // Create memory buffer with source text
        std::unique_ptr<llvm::MemoryBuffer> buffer = llvm::MemoryBuffer::getMemBufferCopy(source, "SIMPLE_BUFFER");
        if (buffer == nullptr)
            throw CGException("Failed to create memory buffer");

        // Remap auxiliary name "string-input" to memory buffer
        PreprocessorOptions& po = compiler.getInvocation().getPreprocessorOpts();
        po.addRemappedFile("string-input", buffer.release());

        HeaderSearchOptions& hso = compiler.getInvocation().getHeaderSearchOpts();
        std::string iClangHeaders = this->findInternalClangCHeaders(_version, hso.ResourceDir);
        if(!iClangHeaders.empty()) {
            hso.AddPath(llvm::StringRef(iClangHeaders), clang::frontend::Angled, false, false);
        }

        for (size_t s = 0; s < _includePaths.size(); s++)
            hso.AddPath(llvm::StringRef(_includePaths[s]), clang::frontend::Angled, false, false);

        // Create and execute the frontend to generate an LLVM bitcode module.
        clang::EmitLLVMOnlyAction action(_context.get());
        if (!compiler.ExecuteAction(action))
            throw CGException("Failed to emit LLVM bitcode");

        std::unique_ptr<llvm::Module> module = action.takeModule();
        if (module == nullptr)
            throw CGException("No module");

        if (_linker.get() == nullptr) {
            _module.reset(module.release());
            _linker.reset(new llvm::Linker(*_module.get()));
        } else {
            if (_linker->linkInModule(std::move(module))) {
                throw CGException("LLVM failed to link module");
            }
        }

        // NO delete module;
        // NO delete invocation;
        //llvm::llvm_shutdown();
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
