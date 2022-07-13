#ifndef CPPAD_CG_CLANG_COMPILER_INCLUDED
#define CPPAD_CG_CLANG_COMPILER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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
 * Clang compiler
 *
 * @author Joao Leal
 */
template<class Base>
class ClangCompiler : public AbstractCCompiler<Base> {
protected:
    std::set<std::string> _bcfiles; // bitcode files
    std::string _version;
public:

    ClangCompiler(const std::string& clangPath = "/usr/bin/clang") :
        AbstractCCompiler<Base>(clangPath) {

        this->_compileFlags.push_back("-O2"); // Optimization level
        this->_compileLibFlags.push_back("-O2"); // Optimization level
        this->_compileLibFlags.push_back("-shared"); // Make shared object
        this->_compileLibFlags.push_back("-rdynamic"); // add all symbols to the dynamic symbol table
    }

    ClangCompiler(const ClangCompiler& orig) = delete;
    ClangCompiler& operator=(const ClangCompiler& rhs) = delete;

    const std::string& getVersion() {
        if(_version.empty()) {
            std::vector<std::string> args {"--version"};
            std::string output;
            system::callExecutable(this->_path, args, &output);

            std::string vv = "version ";
            size_t is = output.find(vv);
            if(is == std::string::npos) {
                throw CGException("Failed to determine Clang version");
            }
            is += vv.size();
            size_t i = is;
            while (i < output.size() && output[i] != ' ' && output[i] != '\n') {
                i++;
            }

            _version = output.substr(is, i - is);
        }
        return _version;
    }

    virtual const std::set<std::string>& getBitCodeFiles() const {
        return _bcfiles;
    }

    virtual void generateLLVMBitCode(const std::map<std::string, std::string>& sources,
                                     JobTimer* timer = nullptr) {
        bool posIndepCode = false;
        this->_compileFlags.push_back("-emit-llvm");
        try {
            this->compileSources(sources, posIndepCode, timer, ".bc", this->_bcfiles);
        } catch (...) {
            this->_compileFlags.pop_back();
            throw;
        }
    }

    /**
     * Creates a dynamic library from a set of object files
     *
     * @param library the path to the dynamic library to be created
     */
    void buildDynamic(const std::string& library,
                      JobTimer* timer = nullptr) override {

#if CPPAD_CG_SYSTEM_APPLE
        std::string linkerName = "-install_name";
#elif CPPAD_CG_SYSTEM_LINUX
        std::string linkerName = "-soname";
#endif
        std::string linkerFlags = "-Wl," + linkerName + "," + system::filenameFromPath(library);
        for (size_t i = 0; i < this->_linkFlags.size(); i++)
            linkerFlags += "," + this->_linkFlags[i];

        std::vector<std::string> args;
        args.insert(args.end(), this->_compileLibFlags.begin(), this->_compileLibFlags.end());
        args.push_back(linkerFlags); // Pass suitable options to linker
        args.push_back("-o"); // Output file name
        args.push_back(library); // Output file name

        for (const std::string& it : this->_ofiles) {
            args.push_back(it);
        }

        if (timer != nullptr) {
            timer->startingJob("'" + library + "'", JobTimer::COMPILING_DYNAMIC_LIBRARY);
        } else if (this->_verbose) {
            std::cout << "building library '" << library << "'" << std::endl;
        }

        system::callExecutable(this->_path, args);

        if (timer != nullptr) {
            timer->finishedJob();
        }
    }

    void cleanup() override {
        // clean up
        for (const std::string& it : _bcfiles) {
            if (remove(it.c_str()) != 0)
                std::cerr << "Failed to delete temporary file '" << it << "'" << std::endl;
        }
        _bcfiles.clear();

        // other files and temporary folder
        AbstractCCompiler<Base>::cleanup();
    }

    virtual ~ClangCompiler() {
        cleanup();
    }

    static std::vector<std::string> parseVersion(const std::string& version) {
        auto vv = explode(version, ".");
        if (vv.size() > 2) {
            auto vv2 = explode(vv[2], "-");
            if (vv2.size() > 1) {
                vv.erase(vv.begin() + 2);
                vv.insert(vv.begin() + 2, vv2.begin(), vv2.end());
            }
        }
        return vv;
    }

protected:

    /**
     * Compiles a single source file into an output file
     * (e.g. object file or bit code file)
     *
     * @param source the content of the source file
     * @param output the compiled output file name (the object file path)
     */
    void compileSource(const std::string& source,
                       const std::string& output,
                       bool posIndepCode) override {
        std::vector<std::string> args;
        args.push_back("-x");
        args.push_back("c"); // C source files
        args.insert(args.end(), this->_compileFlags.begin(), this->_compileFlags.end());
        args.push_back("-c");
        args.push_back("-");
        if (posIndepCode) {
            args.push_back("-fPIC"); // position-independent code for dynamic linking
        }
        args.push_back("-o");
        args.push_back(output);

        system::callExecutable(this->_path, args, nullptr, &source);
    }

    void compileFile(const std::string& path,
                     const std::string& output,
                     bool posIndepCode) override {
        std::vector<std::string> args;
        args.push_back("-x");
        args.push_back("c"); // C source files
        args.insert(args.end(), this->_compileFlags.begin(), this->_compileFlags.end());
        if (posIndepCode) {
            args.push_back("-fPIC"); // position-independent code for dynamic linking
        }
        args.push_back("-c");
        args.push_back(path);
        args.push_back("-o");
        args.push_back(output);

        system::callExecutable(this->_path, args);
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
