#ifndef CPPAD_CG_LANGUAGE_DOT_INDEX_PATTERNS_INCLUDED
#define CPPAD_CG_LANGUAGE_DOT_INDEX_PATTERNS_INCLUDED
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
inline void LanguageDot<Base>::generateNames4RandomIndexPatterns(const std::set<RandomIndexPattern*>& randomPatterns) {
    std::ostringstream os;

    std::set<std::string> usedNames;

    // save existing names so that they are not to overridden 
    // (independent variable names might have already used them)
    for (RandomIndexPattern* ip : randomPatterns) {
        if (!ip->getName().empty()) {
            usedNames.insert(ip->getName());
        }
    }

    // create new names for the index pattern arrays without a name
    size_t c = 0;
    for (RandomIndexPattern* ip : randomPatterns) {
        if (ip->getName().empty()) {
            // new name required
            std::string name;
            do {
                os << _C_STATIC_INDEX_ARRAY << c;
                name = os.str();
                os.str("");
                c++;
            } while (usedNames.find(name) != usedNames.end());

            ip->setName(name);
        }
    }

}

template<class Base>
inline void LanguageDot<Base>::printRandomIndexPatternDeclaration(std::ostringstream& os,
                                                                  const std::set<RandomIndexPattern*>& randomPatterns) {
    if (randomPatterns.empty())
        return;

    _code << "subgraph index {" << _endline;
    _code << "   rank=same" << _endline;
    for (RandomIndexPattern* ip : randomPatterns) {
        os << "idx_" << ip->getName() << "[label=\"";
        if (ip->getType() == IndexPatternType::Random1D) {
            /**
             * 1D
             */
            Random1DIndexPattern* ip1 = static_cast<Random1DIndexPattern*> (ip);
            const std::map<size_t, size_t>& x2y = ip1->getValues();

            std::vector<size_t> y(x2y.rbegin()->first + 1);
            for (const std::pair<size_t, size_t>& p : x2y)
                y[p.first] = p.second;

            printStaticIndexArray(os, ip->getName(), y);
        } else {
            CPPADCG_ASSERT_UNKNOWN(ip->getType() == IndexPatternType::Random2D);
            /**
             * 2D
             */
            Random2DIndexPattern* ip2 = static_cast<Random2DIndexPattern*> (ip);
            printStaticIndexMatrix(os, ip->getName(), ip2->getValues());
        }

        os << "\"]";
        os << _endline;
    }
    _code << "}" << _endline;
}

template<class Base>
void LanguageDot<Base>::printStaticIndexArray(std::ostringstream& os,
                                              const std::string& name,
                                              const std::vector<size_t>& values) {
    os << name << " = [";
    for (size_t i = 0; i < values.size(); i++) {
        if (i > 0)
            os << ",";
        os << values[i];
    }
    os << "]";
}

template<class Base>
void LanguageDot<Base>::printStaticIndexMatrix(std::ostringstream& os,
                                               const std::string& name,
                                               const std::map<size_t, std::map<size_t, size_t> >& values) {
    std::map<size_t, std::map<size_t, size_t> >::const_iterator it;
    std::map<size_t, size_t>::const_iterator ity2z;

    os << name << "= [";
    for (it = values.begin(); it != values.end(); ++it) {
        if (it != values.begin())
            os << it->first << ",\\n";
        os << it->first << ":[";
        size_t y = 0;
        for (ity2z = it->second.begin(); ity2z != it->second.end(); ++ity2z) {
            if (ity2z->first != y) {
                while (ity2z->first != y) {
                    if (y > 0) os << ", ";
                    os << "0";
                    y++;
                }
            }

            if (y > 0) os << ", ";
            os << ity2z->second;

            y++;
        }
        os << "]";
    }
    os << "]";
}

template<class Base>
inline void LanguageDot<Base>::indexPattern2String(std::ostream& os,
                                                      const IndexPattern& ip,
                                                      const OperationNode<Base>& index) {
    indexPattern2String(os, ip,{&index});
}

template<class Base>
inline void LanguageDot<Base>::indexPattern2String(std::ostream& os,
                                                      const IndexPattern& ip,
                                                      const std::vector<const OperationNode<Base>*>& indexes) {
    std::stringstream ss;
    switch (ip.getType()) {
        case IndexPatternType::Linear: // y = x * a + b
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes")
            const auto& lip = static_cast<const LinearIndexPattern&> (ip);
            linearIndexPattern2String(os, lip, *indexes[0]);
            return;
        }
        case IndexPatternType::Sectioned:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes")
            const auto* lip = static_cast<const SectionedIndexPattern*> (&ip);
            const std::map<size_t, IndexPattern*>& sections = lip->getLinearSections();
            size_t sSize = sections.size();
            CPPADCG_ASSERT_UNKNOWN(sSize > 1);

            std::map<size_t, IndexPattern*>::const_iterator its = sections.begin();
            for (size_t s = 0; s < sSize - 1; s++) {
                const IndexPattern* lp = its->second;
                ++its;
                size_t xStart = its->first;

                os << (*indexes[0]->getName()) << " < " << xStart << ")? ";
                indexPattern2String(os, *lp, *indexes[0]);
                os << ": ";
            }
            indexPattern2String(os, *its->second, *indexes[0]);

            return;
        }

        case IndexPatternType::Plane2D: // y = f(x) + f(z)
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() >= 1, "Invalid number of indexes")
            const auto& pip = static_cast<const Plane2DIndexPattern&> (ip);
            bool useParens = pip.getPattern1() != nullptr && pip.getPattern2() != nullptr;

            if (useParens) os << "(";

            if (pip.getPattern1() != nullptr)
                indexPattern2String(os, *pip.getPattern1(), *indexes[0]);

            if (useParens) os << ") + (";

            if (pip.getPattern2() != nullptr)
                indexPattern2String(os, *pip.getPattern2(), *indexes.back());

            if (useParens) os << ")";

            return;
        }
        case IndexPatternType::Random1D:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes")
            const auto& rip = static_cast<const Random1DIndexPattern&> (ip);
            CPPADCG_ASSERT_KNOWN(!rip.getName().empty(), "Invalid name for array")
            os << rip.getName() << "[" << (*indexes[0]->getName()) << "]";
            return;
        }
        case IndexPatternType::Random2D:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 2, "Invalid number of indexes")
            const auto& rip = static_cast<const Random2DIndexPattern&> (ip);
            CPPADCG_ASSERT_KNOWN(!rip.getName().empty(), "Invalid name for array")
            os << rip.getName() <<
                    "[" << (*indexes[0]->getName()) << "]"
                    "[" << (*indexes[1]->getName()) << "]";
            return;
        }
        default:
            CPPADCG_ASSERT_UNKNOWN(false); // should never reach this
            return;
    }
}

template<class Base>
inline void LanguageDot<Base>::linearIndexPattern2String(std::ostream& os,
                                                            const LinearIndexPattern& lip,
                                                            const OperationNode<Base>& index) {
    long dy = lip.getLinearSlopeDy();
    long dx = lip.getLinearSlopeDx();
    long b = lip.getLinearConstantTerm();
    long xOffset = lip.getXOffset();

    if (dy != 0) {
        if (xOffset != 0) {
            os << "(";
        }
        os << (*index.getName());
        if (xOffset != 0) {
            os << " -" << xOffset << ")";
        }

        if (dx != 1) {
            os << "/" << dx;
        }
        if (dy != 1) {
            os << "×" << dy;
        }
    } else if (b == 0) {
        os << "0"; // when dy == 0 and b == 0
    }

    if (b != 0) {
        if (dy != 0)
            os << "+";
        os << b;
    }

}

} // END cg namespace
} // END CppAD namespace

#endif