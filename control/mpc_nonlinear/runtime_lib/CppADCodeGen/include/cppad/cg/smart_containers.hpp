#ifndef CPPAD_CG_SMART_CONTAINERS_INCLUDED
#define CPPAD_CG_SMART_CONTAINERS_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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
 * Smart vector of pointers.
 * Deletes all vector values on destruction.
 */
template<class Base>
class SmartVectorPointer {
public:
    using iterator = typename std::vector<Base*>::iterator;
    using const_iterator = typename std::vector<Base*>::const_iterator;
    using reverse_iterator = typename std::vector<Base*>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<Base*>::const_reverse_iterator;
    std::vector<Base*> v;

    inline SmartVectorPointer() {
    }

    inline SmartVectorPointer(size_t size) :
        v(size) {
    }

    inline SmartVectorPointer(std::vector<Base*>& v_) {
        v.swap(v_);
    }

    inline size_t size() const {
        return v.size();
    }

    inline bool empty() const {
        return v.empty();
    }

    inline void reserve(size_t n) {
        v.reserve(n);
    }

    inline void push_back(Base* x) {
        v.push_back(x);
    }

    inline Base* operator[](size_t n) const {
        return v[n];
    }

    inline Base*& operator[](size_t n) {
        return v[n];
    }

    inline iterator begin() {
        return v.begin();
    }

    inline const_iterator begin() const {
        return v.begin();
    }

    inline iterator end() {
        return v.end();
    }

    inline const_iterator end() const {
        return v.end();
    }

    inline reverse_iterator rbegin() {
        return v.rbegin();
    }

    inline const_reverse_iterator rbegin() const {
        return v.rbegin();
    }

    inline reverse_iterator rend() {
        return v.rend();
    }

    inline const_reverse_iterator rend() const {
        return v.rend();
    }

    inline std::vector<Base*> release() {
        std::vector<Base*> v2;
        v2.swap(v);
        return v2;
    }

    inline virtual ~SmartVectorPointer() {
        for (size_t i = 0; i < v.size(); i++) {
            delete v[i];
        }
    }
};

/**
 * Smart set of pointers.
 * Deletes all set values on destruction.
 */
template<class Base>
class SmartSetPointer {
public:
    using iterator = typename std::set<Base*>::iterator;
    std::set<Base*> s;

    inline SmartSetPointer() {
    }

    inline SmartSetPointer(std::set<Base*>& s_) {
        s.swap(s_);
    }

    inline size_t size() const {
        return s.size();
    }

    inline bool empty() const {
        return s.empty();
    }

    inline iterator begin() const {
        return s.begin();
    }

    inline iterator end() const {
        return s.end();
    }

    inline std::pair<iterator, bool> insert(Base* x) {
        return s.insert(x);
    }

    inline void erase(iterator pos) {
        s.erase(pos);
    }

    inline size_t erase(Base* x) {
        return s.erase(x);
    }

    inline std::set<Base*> release() {
        std::set<Base*> s2;
        s2.swap(s);
        return s2;
    }

    inline virtual ~SmartSetPointer() {
        typename std::set<Base*>::const_iterator it;
        for (it = s.begin(); it != s.end(); ++it) {
            delete *it;
        }
    }
};

/**
 * Smart set of pointers.
 * Deletes all set values on destruction.
 */
template<class Base>
class SmartListPointer {
public:
    using iterator = typename std::list<Base*>::iterator;
    using const_iterator = typename std::list<Base*>::const_iterator;
    std::list<Base*> l;

    inline SmartListPointer() {
    }

    inline SmartListPointer(const std::set<Base*>& l_) {
        l.swap(l_);
    }

    inline size_t size() const {
        return l.size();
    }

    inline bool empty() const {
        return l.empty();
    }

    inline void push_front(Base* x) {
        l.push_front(x);
    }

    inline void pop_front() {
        l.pop_front();
    }

    inline void push_back(Base* x) {
        l.push_back(x);
    }

    inline void pop_back() {
        l.pop_back();
    }

    inline iterator begin() {
        return l.begin();
    }

    inline const_iterator begin() const {
        return l.begin();
    }

    inline iterator end() {
        return l.end();
    }

    inline const_iterator end() const {
        return l.end();
    }

    inline std::list<Base*> release() {
        std::list<Base*> l2;
        l2.swap(l);
        return l2;
    }

    inline virtual ~SmartListPointer() {
        typename std::list<Base*>::const_iterator it;
        for (it = l.begin(); it != l.end(); ++it) {
            delete *it;
        }
    }
};

template<class Key, class Value>
class SmartMapValuePointer {
public:
    using iterator = typename std::map<Key, Value*>::iterator;
    using const_iterator = typename std::map<Key, Value*>::const_iterator;
    using reverse_iterator = typename std::map<Key, Value*>::reverse_iterator;
    using const_reverse_iterator = typename std::map<Key, Value*>::const_reverse_iterator;
    std::map<Key, Value*> m;

    inline size_t size() const {
        return m.size();
    }

    inline bool empty() const {
        return m.empty();
    }

    inline iterator begin() {
        return m.begin();
    }

    inline const_iterator begin() const {
        return m.begin();
    }

    inline iterator end() {
        return m.end();
    }

    inline const_iterator end() const {
        return m.end();
    }

    inline reverse_iterator rbegin() {
        return m.rbegin();
    }

    inline const_reverse_iterator rbegin() const {
        return m.rbegin();
    }

    inline reverse_iterator rend() {
        return m.rend();
    }

    inline const_reverse_iterator rend() const {
        return m.rend();
    }

    inline Value*& operator[](const Key& key) {
        return m[key];
    }

    std::map<Key, Value*> release() {
        std::map<Key, Value*> m2;
        m2.swap(m);
        return m2;
    }

    inline virtual ~SmartMapValuePointer() {
        typename std::map<Key, Value*>::const_iterator it;
        for (it = m.begin(); it != m.end(); ++it) {
            delete it->second;
        }
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
