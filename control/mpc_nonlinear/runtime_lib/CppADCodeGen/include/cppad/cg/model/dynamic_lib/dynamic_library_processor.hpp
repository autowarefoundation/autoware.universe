#ifndef CPPAD_CG_DYNAMIC_LIBRARY_PROCESSOR_INCLUDED
#define CPPAD_CG_DYNAMIC_LIBRARY_PROCESSOR_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *    Copyright (C) 2020 Joao Leal
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
 * Useful class for generating source code for the creation of a dynamic
 * or a static library.
 * 
 * @author Joao Leal
 */
template<class Base>
class DynamicModelLibraryProcessor : public ModelLibraryProcessor<Base> {
protected:
    /**
     * the path of the dynamic or static library to be created
     */
    std::string _libraryName;
    /**
     * a custom extension for the dynamic or static library (e.g. ".so.1")
     */
    std::unique_ptr<const std::string> _customLibExtension;
    /**
     * System dependent custom options
     */
    std::map<std::string, std::string> _options;
public:

    /**
     * Creates a new helper class for the generation of dynamic or static libraries
     * using the C language.
     *
     * @param modelLibGen
     * @param libraryName The path of the dynamic or static library to be created
     *                    (without the extension)
     */
    inline explicit DynamicModelLibraryProcessor(ModelLibraryCSourceGen <Base>& modelLibGen,
                                                 std::string libraryName = "cppad_cg_model") :
            ModelLibraryProcessor<Base>(modelLibGen),
            _libraryName(std::move(libraryName)) {
    }

    virtual ~DynamicModelLibraryProcessor() = default;

    /**
     * @return the path of the dynamic or static library (without the extension)
     */
    inline const std::string& getLibraryName() const {
        return _libraryName;
    }

    /**
     *
     * @param libraryName the path of the dynamic or static library (without the extension)
     */
    inline void setLibraryName(const std::string& libraryName) {
        CPPADCG_ASSERT_KNOWN(!libraryName.empty(), "Library name cannot be empty")

        _libraryName = libraryName;
    }

    /**
     * Provides a custom library extension defined by the user
     * 
     * @return a custom library extension
     */
    inline const std::string* getCustomLibraryExtension() const {
        return _customLibExtension.get();
    }

    /**
     * Defines a custom extension for the library that will be created
     * 
     * @param libraryExtension the custom extension name
     */
    inline void setCustomLibraryExtension(const std::string& libraryExtension) {
        _customLibExtension.reset(new std::string(libraryExtension));
    }

    /**
     * Resets the library extension to the default
     */
    inline void removeCustomLibraryExtension() {
        _customLibExtension.reset();
    }

    /**
     * System dependent custom options
     */
    inline std::map<std::string, std::string>& getOptions() {
        return _options;
    }

    /**
     * System dependent custom options
     */
    inline const std::map<std::string, std::string>& getOptions() const {
        return _options;
    }

    /**
     * Compiles all models and generates a dynamic library.
     * 
     * @param compiler The compiler used to compile the sources and create
     *                 the dynamic library
     * @param loadLib Whether or not to load the dynamic library
     * @return The dynamic library if loadLib is true, nullptr otherwise
     */
    std::unique_ptr<DynamicLib<Base>> createDynamicLibrary(CCompiler<Base>& compiler,
                                                           bool loadLib = true) {
        // backup output format so that it can be restored
        OStreamConfigRestore coutb(std::cout);

        this->modelLibraryHelper_->startingJob("", JobTimer::DYNAMIC_MODEL_LIBRARY);

        const std::map<std::string, ModelCSourceGen < Base>*>&models = this->modelLibraryHelper_->getModels();
        try {
            for (const auto& p : models) {
                const std::map<std::string, std::string>& modelSources = this->getSources(*p.second);

                this->modelLibraryHelper_->startingJob("", JobTimer::COMPILING_FOR_MODEL);
                compiler.compileSources(modelSources, true, this->modelLibraryHelper_);
                this->modelLibraryHelper_->finishedJob();
            }

            const std::map<std::string, std::string>& sources = this->getLibrarySources();
            compiler.compileSources(sources, true, this->modelLibraryHelper_);

            const std::map<std::string, std::string>& customSource = this->modelLibraryHelper_->getCustomSources();
            compiler.compileSources(customSource, true, this->modelLibraryHelper_);

            std::string libname = _libraryName;
            if (_customLibExtension != nullptr)
                libname += *_customLibExtension;
            else
                libname += system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;

            compiler.buildDynamic(libname, this->modelLibraryHelper_);

        } catch (...) {
            compiler.cleanup();
            throw;
        }
        compiler.cleanup();

        this->modelLibraryHelper_->finishedJob();

        if (loadLib)
            return loadDynamicLibrary();
        else
            return std::unique_ptr<DynamicLib<Base>> (nullptr);
    }

    /**
     * Compiles all models and generates a static library.
     * 
     * @param compiler The compiler used to compile the sources
     * @param ar The archiver used to assemble the compiled source into a
     *           static library
     * @param posIndepCode Whether or not to compile the source 
     *                     with position independent code (static libraries 
     *                     typically do not use this feature)
     */

    void createStaticLibrary(CCompiler <Base>& compiler,
                             Archiver& ar,
                             bool posIndepCode) {
        // backup output format so that it can be restored
        OStreamConfigRestore coutb(std::cout);

        this->modelLibraryHelper_->startingJob("", JobTimer::STATIC_MODEL_LIBRARY);

        const std::map<std::string, ModelCSourceGen<Base>*>& models = this->modelLibraryHelper_->getModels();
        try {
            for (const auto& p : models) {
                const std::map<std::string, std::string>& modelSources = this->getSources(*p.second);

                this->modelLibraryHelper_->startingJob("", JobTimer::COMPILING_FOR_MODEL);
                compiler.compileSources(modelSources, posIndepCode, this->modelLibraryHelper_);
                this->modelLibraryHelper_->finishedJob();
            }

            const std::map<std::string, std::string>& sources = this->getLibrarySources();
            compiler.compileSources(sources, posIndepCode, this->modelLibraryHelper_);

            const std::map<std::string, std::string>& customSource = this->modelLibraryHelper_->getCustomSources();
            compiler.compileSources(customSource, posIndepCode, this->modelLibraryHelper_);

            std::string libname = _libraryName;
            if (_customLibExtension != nullptr)
                libname += *_customLibExtension;
            else
                libname += system::SystemInfo<>::STATIC_LIB_EXTENSION;

            ar.create(libname, compiler.getObjectFiles(), this->modelLibraryHelper_);
        } catch (...) {
            compiler.cleanup();
            throw;
        }
        compiler.cleanup();

        this->modelLibraryHelper_->finishedJob();
    }


protected:

    virtual std::unique_ptr<DynamicLib<Base>> loadDynamicLibrary();

};

} // END cg namespace
} // END CppAD namespace

#endif
