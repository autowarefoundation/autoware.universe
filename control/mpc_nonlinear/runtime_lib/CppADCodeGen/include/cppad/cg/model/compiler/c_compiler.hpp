#ifndef CPPAD_CG_C_COMPILER_INCLUDED
#define CPPAD_CG_C_COMPILER_INCLUDED
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
class CCompiler {
public:

    /**
     * Provides the path to a temporary folder that should not exist
     * (it will be deleted after the dynamic library is created)
     *
     * @return path to a temporary folder.
     */
    virtual const std::string& getTemporaryFolder() const = 0;

    /**
     * Defines the path to a temporary folder that should not exist
     * (it will be deleted after the dynamic library is created)
     *
     * @param tmpFolder path to a temporary folder.
     */
    virtual void setTemporaryFolder(const std::string& tmpFolder) = 0;

    virtual bool isSaveToDiskFirst() const = 0;

    virtual void setSaveToDiskFirst(bool saveToDiskFirst) = 0;

    /**
     * Provides the path to a folder where the source files should be created
     * when the option to save files to disk is active.
     *
     * @return path to a folder.
     */
    virtual const std::string& getSourcesFolder() const = 0;

    /**
     * Defines the path to a folder where the source files should be created
     * when the option to save files to disk is active.
     *
     * @param srcFolder path to the folder.
     */
    virtual void setSourcesFolder(const std::string& srcFolder) = 0;

    virtual const std::set<std::string>& getObjectFiles() const = 0;

    virtual const std::set<std::string>& getSourceFiles() const = 0;

    virtual bool isVerbose() const = 0;

    virtual void setVerbose(bool verbose) = 0;

    /**
     * Compiles the provided C source code.
     *
     * @param sources maps the names to the content of the source files
     * @param posIndepCode whether or not to create position-independent
     *                     code for dynamic linking
     */
    virtual void compileSources(const std::map<std::string, std::string>& sources,
                                bool posIndepCode,
                                JobTimer* timer = nullptr) = 0;

    /**
     * Creates a dynamic library from the previously compiled object files
     *
     * @param library the path to the dynamic library to be created
     */
    virtual void buildDynamic(const std::string& library,
                              JobTimer* timer = nullptr) = 0;

    /**
     * Deletes the previously compiled object files and clears of files
     * to include in a dynamic library
     */
    virtual void cleanup() = 0;

    inline virtual ~CCompiler() = default;

};

} // END cg namespace
} // END CppAD namespace

#endif
