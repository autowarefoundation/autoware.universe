#ifndef CPPAD_CG_LANGUAGE_DOT_ARRAYS_INCLUDED
#define CPPAD_CG_LANGUAGE_DOT_ARRAYS_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

template<class Base>
std::string LanguageDot<Base>::printArrayCreationOp(OperationNode<Base>& array) {
    CPPADCG_ASSERT_KNOWN(array.getArguments().size() > 0, "Invalid number of arguments for array creation operation")
    const std::vector<Argument<Base> >& args = array.getArguments();
    const size_t argSize = args.size();

    _ss.str("");
    _ss << "array[" << args.size() << "]";
    std::string name = printNodeDeclaration(array, _ss);

    for (size_t i = 0; i < argSize; i++) {
        // try to use a loop for element assignment
        size_t newI = printArrayCreationUsingLoop(name, array, i, nullptr);

        // print elements not assign in a loop
        if (newI == i) {
            // individual element assignment
            std::string aName = print(args[i]);

            printEdge(aName, name, std::to_string(i)); // std::to_string(i)
            _code << _endline;

        } else {
            i = newI - 1;
        }
    }

    return name;
}

template<class Base>
std::string LanguageDot<Base>::printSparseArrayCreationOp(OperationNode<Base>& array) {

    const std::vector<size_t>& info = array.getInfo();
    CPPADCG_ASSERT_KNOWN(!info.empty(), "Invalid number of information elements for sparse array creation operation")

    const std::vector<Argument<Base> >& args = array.getArguments();
    const size_t argSize = args.size();

    CPPADCG_ASSERT_KNOWN(info.size() == argSize + 1, "Invalid number of arguments for sparse array creation operation")

    _ss.str("");
    _ss << "sparse[" << info[0] << "]"; // nnz: args.size()
    std::string name = printNodeDeclaration(array, _ss);

    if (argSize == 0)
        return name; // empty array

    for (size_t i = 0; i < argSize; i++) {
        // try to use a loop for element assignment
        size_t newI = printArrayCreationUsingLoop(name, array, i, &info[i]);

        // print element values not assign in a loop
        if (newI == i) {
            // individual element assignment
            std::string aName = print(args[i]);

            printEdge(aName, name, std::to_string(info[i + 1])); // std::to_string(i)
            _code << _endline;

        } else {
            i = newI - 1;
        }
    }

    return name;
}

template<class Base>
inline size_t LanguageDot<Base>::printArrayCreationUsingLoop(const std::string& arrayName,
                                                             const OperationNode<Base>& array,
                                                             size_t starti,
                                                             const size_t* indexes) {

    const std::vector<Argument<Base> >& args = array.getArguments();
    const size_t argSize = args.size();
    size_t i = starti + 1;

    /**
     * constant value?
     */
    if(args[starti].getParameter() == nullptr)
        return starti;

    const Base& value = *args[starti].getParameter();
    for (; i < argSize; i++) {
        if (args[i].getParameter() == nullptr ||
            *args[i].getParameter() != value) {
            break; // not the same constant value
        }

        if (indexes != nullptr && i - starti != indexes[i] - indexes[starti])
            break; // not the same constant value
    }

    if (i - starti < 3)
        return starti;

    std::string aName = print(args[starti]);

    /**
     * print the loop
     */
    if (indexes != nullptr)
        printEdge(aName, arrayName, std::to_string(indexes[starti]) + "..." + std::to_string(indexes[i]));
    else
        printEdge(aName, arrayName, std::to_string(starti) + "..." + std::to_string(i));
    _code << _endline;

    return i;
}

template<class Base>
std::string LanguageDot<Base>::printArrayElementOp(OperationNode<Base>& op) {
    CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for array element operation")
    CPPADCG_ASSERT_KNOWN(op.getArguments()[0].getOperation() != nullptr, "Invalid argument for array element operation")
    CPPADCG_ASSERT_KNOWN(op.getInfo().size() == 1, "Invalid number of information indexes for array element operation")

    std::string name = makeNodeName(op);

    _ss.str("");
    _ss << "[" << op.getInfo()[0] << "]";
    printEdges(name, op, std::vector<std::string>{}, std::vector<std::string>{_ss.str()});

    return name;
}

} // END cg namespace
} // END CppAD namespace

#endif