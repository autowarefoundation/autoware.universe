#ifndef CPPAD_CG_INDEPENDENT_SORTER_INCLUDED
#define CPPAD_CG_INDEPENDENT_SORTER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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
 * Class used to sort independent variable nodes in ascending order 
 * according to their index (saved in the info vector of the nodes)
 */
template<class Base>
class IndependentNodeSorter {
public:

    /**
     * @return true if the first node goes before the second node
     */
    bool operator()(const OperationNode<Base>* node1,
                    const OperationNode<Base>* node2) const {
        CPPADCG_ASSERT_UNKNOWN(node1 == nullptr || node1->getInfo().size() == 1);
        CPPADCG_ASSERT_UNKNOWN(node2 == nullptr || node2->getInfo().size() == 1);
        CPPADCG_ASSERT_UNKNOWN(node1 == nullptr || node1->getOperationType() == CGOpCode::Inv);
        CPPADCG_ASSERT_UNKNOWN(node2 == nullptr || node2->getOperationType() == CGOpCode::Inv);

        // some variables are not used in all iterations
        if (node1 == nullptr) {
            if (node2 == nullptr) {
                return false;
            }
            return true;
        } else if (node2 == nullptr) {
            return false;
        }

        size_t index1 = node1->getInfo()[0];
        size_t index2 = node2->getInfo()[0];
        if (index1 < index2)
            return true;
        else //if (index1 >= index2)
            return false;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
