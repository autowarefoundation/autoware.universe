#ifndef CPPAD_CG_LLVM_MODEL_LIBRARY_PROCESSOR_INCLUDED
#define CPPAD_CG_LLVM_MODEL_LIBRARY_PROCESSOR_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
 *    Copyright (C) 2019 Joao Leal
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
 * Useful class for generating a JIT evaluated model library.
 * 
 * @author Joao Leal
 */
template<class Base>
class LlvmModelLibraryProcessor : public LlvmBaseModelLibraryProcessor<Base> {
protected:
    const std::string _version;
    std::vector<std::string> _includePaths;
    std::unique_ptr<llvm::Linker> _linker;
    std::unique_ptr<llvm::LLVMContext> _context;
public:

    /**
     * 
     * @param modelLibraryHelper
     */
    LlvmModelLibraryProcessor(ModelLibraryCSourceGen<Base>& modelLibraryHelper) :
            LlvmBaseModelLibraryProcessor<Base>(modelLibraryHelper),
            _version("3.4") {
    }

    virtual ~LlvmModelLibraryProcessor() = default;

    /**
     * @return The version of LLVM (and Clang).
     */
    inline const std::string& getVersion() const {
        return _version;
    }

    inline void setIncludePaths(const std::vector<std::string>& includePaths) {
        _includePaths = includePaths;
    }

    inline const std::vector<std::string>& getIncludePaths() const {
        return _includePaths;
    }

    std::unique_ptr<LlvmModelLibrary<Base>> create() {
        ClangCompiler<Base> clang;
        return create(clang);
    }

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
            const std::set<std::string>& bcFiles = this->createBitCode(clang, "3.4");

            /**
             * Load bit code and create a single module
             */
            llvm::InitializeAllTargets();
            llvm::InitializeAllAsmPrinters();

            _context.reset(new llvm::LLVMContext());

            for (const std::string& itbc : bcFiles) {
                // load bitcode file
                OwningPtr<MemoryBuffer> buffer;

                error_code ec = MemoryBuffer::getFile(itbc, buffer);
                if (buffer.get() == nullptr)
                    throw CGException(ec.message());

                // create the module
                std::string errMsg;
                Module* module = llvm::ParseBitcodeFile(buffer.get(), *_context.get(), &errMsg);
                if(module == nullptr)
                    throw CGException("Failed to create LLVM bitcode: ", errMsg);

                // link modules together
                if (_linker.get() == nullptr) {
                    _linker.reset(new llvm::Linker(module)); // module not destroyed
                } else {
                    if (_linker->linkInModule(module, &errMsg)) { // module destroyed
                        throw CGException(errMsg);
                    }
                }
            }

            llvm::InitializeNativeTarget();

            // voila
            lib.reset(new LlvmModelLibrary3_4<Base>(_linker->getModule(), _context.release()));

        } catch (...) {
            clang.cleanup();
            throw;
        }
        clang.cleanup();

        this->modelLibraryHelper_->finishedJob();

        return lib;
    }

    static inline std::unique_ptr<LlvmModelLibrary<Base>> create(ModelLibraryCSourceGen<Base>& modelLibraryHelper) {
        LlvmModelLibraryProcessor<Base> p(modelLibraryHelper);
        return p.create();
    }

protected:
#if 0

    /**
     * Creates LLVM modules in a separate process.
     */
    static void createnPrintModule() {
        using namespace llvm;
        using namespace clang;

        /**
         * load source file
         */
        std::string source;
        std::cin >> source;

        static const char* argv [] = {"program", "-Wall", "-x", "c", "string-input"};
        static const int argc = sizeof (argv) / sizeof (argv[0]);

        IntrusiveRefCntPtr<DiagnosticOptions> diagOpts = new DiagnosticOptions();
        TextDiagnosticPrinter *diagClient = new TextDiagnosticPrinter(llvm::errs(), &*diagOpts); // will be owned by diags
        IntrusiveRefCntPtr<DiagnosticIDs> diagID(new DiagnosticIDs());
        IntrusiveRefCntPtr<DiagnosticsEngine> diags(new DiagnosticsEngine(diagID, &*diagOpts, diagClient));

        ArrayRef<const char *> args(argv + 1, // skip program name
                                    argc - 1);
        std::unique_ptr<CompilerInvocation> invocation(createInvocationFromCommandLine(args, diags));
        if (invocation.get() == nullptr)
            throw CGException("Failed to create compiler invocation");
        CompilerInvocation::setLangDefaults(*invocation->getLangOpts(), IK_C,
                                            LangStandard::lang_unspecified);
        invocation->getFrontendOpts().DisableFree = false; // make sure we free memory (by default it does not)

        // Create a compiler instance to handle the actual work.
        CompilerInstance compiler;
        compiler.setInvocation(invocation.release());

        // Create the compilers actual diagnostics engine.
        compiler.createDiagnostics();
        if (!compiler.hasDiagnostics())
            throw CGException("No diagnostics");

        // Create memory buffer with source text
        llvm::MemoryBuffer * buffer = llvm::MemoryBuffer::getMemBufferCopy(source, "SIMPLE_BUFFER");
        if (buffer == nullptr)
            throw CGException("Failed to create memory buffer");

        // Remap auxiliary name "string-input" to memory buffer
        PreprocessorOptions& po = compiler.getInvocation().getPreprocessorOpts();
        po.addRemappedFile("string-input", buffer);

        HeaderSearchOptions& hso = compiler.getInvocation().getHeaderSearchOpts();
        for (size_t s = 0; s < _includePaths.size(); s++)
            hso.AddPath(llvm::StringRef(_includePaths[s]), clang::frontend::Angled, true, false);

        // Create and execute the frontend to generate an LLVM bitcode module.
        OwningPtr<CodeGenAction> action(new clang::EmitLLVMOnlyAction(_context.get()));

        if (!compiler.ExecuteAction(*action))
            throw CGException("Failed to emit LLVM bitcode");

        llvm::Module* module = action->takeModule();
        if (module == nullptr)
            throw CGException("No module");

        /**
         * Print out the IR
         */
        //std::cout << *module;
        raw_fd_ostream os(STDOUT_FILENO, true);
        module->print(os);
        delete module;
    }
#endif
};

} // END cg namespace
} // END CppAD namespace

#endif
