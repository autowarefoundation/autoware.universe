#ifndef CPPAD_CG_CODE_HANDLER_LOOPS_INCLUDED
#define CPPAD_CG_CODE_HANDLER_LOOPS_INCLUDED
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

template<class Base>
const std::map<size_t, LoopModel<Base>*>& CodeHandler<Base>::getLoops() const {
    return _loops.loopModels;
}

template<class Base>
inline LoopModel<Base>* CodeHandler<Base>::getLoop(size_t loopId) const {
    return _loops.getLoop(loopId);
}

template<class Base>
inline size_t CodeHandler<Base>::addLoopDependentIndexPattern(IndexPattern& jacPattern) {
    return _loops.addDependentIndexPattern(jacPattern);
}

template<class Base>
inline void CodeHandler<Base>::manageLoopDependentIndexPattern(const IndexPattern* pattern) {
    _loops.manageDependentIndexPattern(pattern);
}

template<class Base>
inline size_t CodeHandler<Base>::addLoopIndependentIndexPattern(IndexPattern& pattern,
                                                                size_t hint) {
    return _loops.addIndependentIndexPattern(pattern, hint);
}

template<class Base>
inline void CodeHandler<Base>::LoopData::prepare4NewSourceGen() {
    indexes.clear();
    indexRandomPatterns.clear();
    outerVars.clear();
    depth = -1;
    startEvalOrder.clear();
    endNodes.clear();

    endNodes.reserve(loopModels.size());
}

template<class Base>
inline void CodeHandler<Base>::LoopData::reset() {
    loopModels.clear();
    indexes.clear();
    indexRandomPatterns.clear();
    dependentIndexPatterns.clear();
    independentIndexPatterns.clear();
    endNodes.clear();

    for (const IndexPattern* itip : dependentIndexPatternManaged) {
        delete itip;
    }
    dependentIndexPatternManaged.clear();
}

template<class Base>
inline const std::string* CodeHandler<Base>::LoopData::getLoopName(size_t id) const {
    typename std::map<size_t, LoopModel<Base>*>::const_iterator it;
    it = loopModels.find(id);
    if (it != loopModels.end())
        return &(it->second->atomic_name());
    else
        return nullptr;
}

template<class Base>
void CodeHandler<Base>::LoopData::registerModel(LoopModel<Base>& loop) {
    loopModels[loop.getLoopId()] = &loop;
}

template<class Base>
LoopModel<Base>* CodeHandler<Base>::LoopData::getLoop(size_t loopId) const {
    typename std::map<size_t, LoopModel<Base>*>::const_iterator it = loopModels.find(loopId);
    if (it != loopModels.end()) {
        return it->second;
    }

    return nullptr;
}

template<class Base>
size_t CodeHandler<Base>::LoopData::addDependentIndexPattern(IndexPattern& pattern) {
    size_t size = dependentIndexPatterns.size();
    if (dependentIndexPatterns.capacity() == size) {
        dependentIndexPatterns.reserve((size * 3) / 2 + 1);
    }
    dependentIndexPatterns.push_back(&pattern);

    return size;
}

template<class Base>
void CodeHandler<Base>::LoopData::manageDependentIndexPattern(const IndexPattern* pattern) {
    size_t sizeM = dependentIndexPatternManaged.size();
    if (dependentIndexPatternManaged.capacity() == sizeM) {
        dependentIndexPatternManaged.reserve((sizeM * 3) / 2 + 1);
    }
    dependentIndexPatternManaged.push_back(pattern);
}

template<class Base>
size_t CodeHandler<Base>::LoopData::addIndependentIndexPattern(IndexPattern& pattern, size_t hint) {
    size_t size = independentIndexPatterns.size();
    if (hint < size && independentIndexPatterns[hint] == &pattern) {
        return hint;
    }
    if (independentIndexPatterns.capacity() == size) {
        independentIndexPatterns.reserve((size * 3) / 2 + 1);
    }
    independentIndexPatterns.push_back(&pattern);

    return size;
}

template<class Base>
void CodeHandler<Base>::LoopData::addLoopEndNode(OperationNode<Base>& node) {
    CPPADCG_ASSERT_UNKNOWN(node.getOperationType() == CGOpCode::LoopEnd);
    LoopEndOperationNode<Base>& loopEnd = static_cast<LoopEndOperationNode<Base>&> (node);
    endNodes.push_back(&loopEnd);
}

} // END cg namespace
} // END CppAD namespace

#endif