#ifndef CPPAD_CG_LANG_LATEX_CUSTOM_VAR_NAME_GEN_INCLUDED
#define CPPAD_CG_LANG_LATEX_CUSTOM_VAR_NAME_GEN_INCLUDED
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
 * Creates variables names for the source code using a list of provided
 * custom names.
 *
 * @author Joao Leal
 */
template<class Base>
class LangLatexCustomVariableNameGenerator : public LangLatexDefaultVariableNameGenerator<Base> {
protected:
    using Super = LangLatexDefaultVariableNameGenerator<Base>;
    // the custom names for the dependent variables
    const std::vector<std::string> depNames_;
    // the custom names for the independent variables
    const std::vector<std::string> indepNames_;
public:

    LangLatexCustomVariableNameGenerator(const std::vector<std::string>& depNames,
                                         const std::vector<std::string>& indepNames,
                                         const std::string& depName = "y",
                                         const std::string& indepName = "x",
                                         const std::string& tmpName = "v",
                                         const std::string& tmpArrayName = "\\mathbf{a}",
                                         const std::string& tmpSparseArrayName = "\\mathbf{s}") :
        LangLatexDefaultVariableNameGenerator<Base>(depName, indepName, tmpName, tmpArrayName, tmpSparseArrayName),
        depNames_(depNames),
        indepNames_(indepNames) {
    }

    inline virtual ~LangLatexCustomVariableNameGenerator() = default;

    std::string generateDependent(size_t index) override {
        if (index < depNames_.size() && !depNames_[index].empty()) {
            return depNames_[index];
        } else {
            return Super::generateDependent(index);
        }
    }

    std::string generateIndependent(const OperationNode<Base>& independent,
                                    size_t id) override {
        size_t index = id - 1;
        if (index < indepNames_.size() && !indepNames_[index].empty()) {
            return indepNames_[index];
        } else {
            return Super::generateIndependent(independent, id);
        }
    }

    bool isConsecutiveInIndepArray(const OperationNode<Base>& indepFirst,
                                   size_t idFirst,
                                   const OperationNode<Base>& indepSecond,
                                   size_t idSecond) override {
        size_t index1 = idFirst - 1;
        size_t index2 = idSecond - 1;

        if ((index1 > indepNames_.size() || indepNames_[index1].empty()) &&
                (index2 > indepNames_.size() || indepNames_[index2].empty())) {
            return index1 + 1 == index2;
        } else {
            return false; // individual names used (not elements of arrays)
        }
    }

    bool isInSameIndependentArray(const OperationNode<Base>& indep1,
                                  size_t id1,
                                  const OperationNode<Base>& indep2,
                                  size_t id2) override {
        size_t index1 = id1 - 1;
        size_t index2 = id2 - 1;

        return (index1 > indepNames_.size() || indepNames_[index1].empty()) &&
                (index2 > indepNames_.size() || indepNames_[index2].empty());
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
