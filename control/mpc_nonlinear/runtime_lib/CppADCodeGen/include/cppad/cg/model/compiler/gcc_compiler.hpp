#ifndef CPPAD_CG_GCC_COMPILER_INCLUDED
#define CPPAD_CG_GCC_COMPILER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
 * C compiler class used to create a dynamic library
 *
 * @author Joao Leal
 */
template<class Base>
class GccCompiler : public AbstractCCompiler<Base> {
public:

    GccCompiler(const std::string& gccPath = "/usr/bin/gcc") :
        AbstractCCompiler<Base>(gccPath) {

        this->_compileFlags.push_back("-O2"); // Optimization level
        this->_compileLibFlags.push_back("-O2"); // Optimization level
        this->_compileLibFlags.push_back("-shared"); // Make shared object
        this->_compileLibFlags.push_back("-rdynamic"); // add all symbols to the dynamic symbol table
    }

    GccCompiler(const GccCompiler& orig) = delete;
    GccCompiler& operator=(const GccCompiler& rhs) = delete;

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

    virtual ~GccCompiler() = default;

protected:

    /**
     * Compiles a single source file into an object file
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
