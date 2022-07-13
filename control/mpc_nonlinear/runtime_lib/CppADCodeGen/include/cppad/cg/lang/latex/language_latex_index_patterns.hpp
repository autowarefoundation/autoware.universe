#ifndef CPPAD_CG_LANGUAGE_LATEX_INDEX_PATTERNS_INCLUDED
#define CPPAD_CG_LANGUAGE_LATEX_INDEX_PATTERNS_INCLUDED
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

namespace CppAD {
namespace cg {

template<class Base>
inline void LanguageLatex<Base>::generateNames4RandomIndexPatterns(const std::set<RandomIndexPattern*>& randomPatterns) {
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
inline void LanguageLatex<Base>::printRandomIndexPatternDeclaration(std::ostringstream& os,
                                                                    const std::string& indentation,
                                                                    const std::set<RandomIndexPattern*>& randomPatterns) {
    for (RandomIndexPattern* ip : randomPatterns) {
        if (ip->getType() == IndexPatternType::Random1D) {
            /**
             * 1D
             */
            auto* ip1 = static_cast<Random1DIndexPattern*> (ip);
            const std::map<size_t, size_t>& x2y = ip1->getValues();

            std::vector<size_t> y(x2y.rbegin()->first + 1);
            for (const auto& p : x2y)
                y[p.first] = p.second;

            os << indentation;
            printStaticIndexArray(os, ip->getName(), y);
        } else {
            CPPADCG_ASSERT_UNKNOWN(ip->getType() == IndexPatternType::Random2D)
            /**
             * 2D
             */
            auto* ip2 = static_cast<Random2DIndexPattern*> (ip);
            os << indentation;
            printStaticIndexMatrix(os, ip->getName(), ip2->getValues());
        }
    }
}

template<class Base>
void LanguageLatex<Base>::printStaticIndexArray(std::ostringstream& os,
                                                const std::string& name,
                                                const std::vector<size_t>& values) {
    os << name << " = \\left\\{";
    if (!values.empty()) {
        os << values[0];
        for (size_t i = 1; i < values.size(); i++) {
            os << ", " << values[i];
        }
    }
    os << "\\right\\}" << _endEq << " % size: " << values.size() << _endline;
}

template<class Base>
void LanguageLatex<Base>::printStaticIndexMatrix(std::ostringstream& os,
                                                 const std::string& name,
                                                 const std::map<size_t, std::map<size_t, size_t> >& values) {
    size_t m = 0;
    size_t n = 0;

    std::map<size_t, std::map<size_t, size_t> >::const_iterator it;
    std::map<size_t, size_t>::const_iterator ity2z;

    if (!values.empty()) {
        m = values.rbegin()->first + 1;

        for (it = values.begin(); it != values.end(); ++it) {
            if (!it->second.empty())
                n = std::max<size_t>(n, it->second.rbegin()->first + 1);
        }
    }

    os << name << " = \\left\\{";
    size_t x = 0;
    for (it = values.begin(); it != values.end(); ++it) {
        if (it->first != x) {
            while (it->first != x) {
                os << "{},";
                x++;
            }
        }

        os << "{";

        size_t y = 0;
        for (ity2z = it->second.begin(); ity2z != it->second.end(); ++ity2z) {
            if (ity2z->first != y) {
                while (ity2z->first != y) {
                    os << "0,";
                    y++;
                }
            }

            os << ity2z->second;
            if (ity2z->first != it->second.rbegin()->first) os << ",";

            y++;
        }

        os << "}";
        if (it->first != values.rbegin()->first) os << ",";

        x++;
    }
    os << "\\right\\}" << _endEq << "% size: " << m << " x " << n << _endline;
}

template<class Base>
inline std::string LanguageLatex<Base>::indexPattern2String(const IndexPattern& ip,
                                                            const OperationNode<Base>& index) {
    return indexPattern2String(ip,{&index});
}

template<class Base>
inline std::string LanguageLatex<Base>::indexPattern2String(const IndexPattern& ip,
                                                            const std::vector<const OperationNode<Base>*>& indexes) {
    std::stringstream ss;
    switch (ip.getType()) {
        case IndexPatternType::Linear: // y = x * a + b
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes")
            const auto& lip = static_cast<const LinearIndexPattern&> (ip);
            return linearIndexPattern2String(lip, *indexes[0]);
        }
        case IndexPatternType::Sectioned:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes")
            const auto* lip = static_cast<const SectionedIndexPattern*> (&ip);
            const std::map<size_t, IndexPattern*>& sections = lip->getLinearSections();
            size_t sSize = sections.size();
            CPPADCG_ASSERT_UNKNOWN(sSize > 1)

            auto its = sections.begin();
            for (size_t s = 0; s < sSize - 1; s++) {
                const IndexPattern* lp = its->second;
                ++its;
                size_t xStart = its->first;

                ss << "\\left(" << (*indexes[0]->getName()) << "<" << xStart << "\\right)? "
                        << indexPattern2String(*lp, *indexes[0]) << ": ";
            }
            ss << indexPattern2String(*its->second, *indexes[0]);

            return ss.str();
        }

        case IndexPatternType::Plane2D: // y = f(x) + f(z)
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() >= 1, "Invalid number of indexes")
            std::string indexExpr;
            const auto& pip = static_cast<const Plane2DIndexPattern&> (ip);
            bool useParens = pip.getPattern1() != nullptr && pip.getPattern2() != nullptr;

            if (useParens) indexExpr += "\\left(";

            if (pip.getPattern1() != nullptr)
                indexExpr += indexPattern2String(*pip.getPattern1(), *indexes[0]);

            if (useParens) indexExpr += "\\right) + \\left(";

            if (pip.getPattern2() != nullptr)
                indexExpr += indexPattern2String(*pip.getPattern2(), *indexes.back());

            if (useParens) indexExpr += "\\right)";

            return indexExpr;
        }
        case IndexPatternType::Random1D:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 1, "Invalid number of indexes")
            const auto& rip = static_cast<const Random1DIndexPattern&> (ip);
            CPPADCG_ASSERT_KNOWN(!rip.getName().empty(), "Invalid name for array")
            return rip.getName() + "\\left[" + (*indexes[0]->getName()) + "\\right]";
        }
        case IndexPatternType::Random2D:
        {
            CPPADCG_ASSERT_KNOWN(indexes.size() == 2, "Invalid number of indexes")
            const auto& rip = static_cast<const Random2DIndexPattern&> (ip);
            CPPADCG_ASSERT_KNOWN(!rip.getName().empty(), "Invalid name for array")
            return rip.getName() + "\\left[" + (*indexes[0]->getName()) + "\\right]\\left[" + (*indexes[1]->getName()) + "\\right]";
        }
        default:
            CPPADCG_ASSERT_UNKNOWN(false); // should never reach this
            return "";
    }
}

template<class Base>
inline std::string LanguageLatex<Base>::linearIndexPattern2String(const LinearIndexPattern& lip,
                                                                  const OperationNode<Base>& index) {
    long dy = lip.getLinearSlopeDy();
    long dx = lip.getLinearSlopeDx();
    long b = lip.getLinearConstantTerm();
    long xOffset = lip.getXOffset();

    std::stringstream ss;
    if (dy != 0) {
        if (xOffset != 0) {
            ss << "\\left(";
        }
        ss << (*index.getName());
        if (xOffset != 0) {
            ss << " - " << xOffset << "\\right)";
        }

        if (dx != 1) {
            ss << " / " << dx;
        }
        if (dy != 1) {
            ss << " \\cdot " << dy;
        }
    } else if (b == 0) {
        ss << "0"; // when dy == 0 and b == 0
    }

    if (b != 0) {
        if (dy != 0)
            ss << " + ";
        ss << b;
    }
    return ss.str();
}

} // END cg namespace
} // END CppAD namespace

#endif