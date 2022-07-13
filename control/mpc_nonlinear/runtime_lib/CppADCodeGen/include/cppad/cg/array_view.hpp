#ifndef CPPAD_CG_ARRAY_VIEW_INCLUDED
#define CPPAD_CG_ARRAY_VIEW_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

# include <cstddef>
# include <iostream>
# include <limits>

namespace CppAD {
namespace cg {

/**
 * A simple wrapper for C arrays.
 * It does not own the data array.
 */
template<class Type>
class ArrayView {
public:
    using value_type = Type;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using reference = value_type&;
    using const_reference = const Type&;
    using iterator = value_type*;
    using const_iterator = const value_type*;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using size_type = size_t;
    using difference_type = ptrdiff_t;
private:
    /**
     * The externally created array
     */
    pointer _data;
    /**
     * The number of elements in the array
     */
    size_type _length;
public:
    /**
     * Default empty constructor for arrays with no elements
     */
    inline ArrayView() :
            _data(nullptr),
            _length(0) {
    }

    /**
     * Creates a wrapper for an existing array.
     *
     * @param array pointer to the first element of the array
     * @param n  size of the array
     */
    inline ArrayView(pointer array,
                     size_type n) :
            _data(array),
            _length(n) {
        CPPAD_ASSERT_KNOWN(array != nullptr || n == 0, "ArrayView: null array with a non-zero size");
    }

    /**
     * Creates a wrapper from a vector.
     * It is expected that the vector is not resized nor deleted while using
     * this wrapper.
     *
     * @param vector the vector to wrap
     */
    inline ArrayView(std::vector<value_type>& vector) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Creates a wrapper from a vector.
     * It is expected that the vector is not resized nor deleted while using
     * this wrapper.
     *
     * @param vector the vector to wrap
     */
    inline ArrayView(CppAD::vector<value_type>& vector) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Creates a wrapper from an std::array.
     * It is expected that std::array is not deleted while using this wrapper.
     *
     * @param array the std::array to wrap
     */
    template<std::size_t S>
    inline ArrayView(std::array<value_type, S>& array) :
            _data(array.data()),
            _length(S) {
    }

    /**
     * Creates a wrapper from an std::valarray.
     * It is expected that std::valarray is not deleted while using this wrapper.
     *
     * @param array the valarray to wrap
     */
    inline ArrayView(std::valarray<value_type>& array) :
            _data(array.size() > 0 ? &array[0] : nullptr),
            _length(array.size()) {
    }

    /**
     * Creates an ArrayView for a const data type from another ArrayView with
     * a non-const data type.
     *
     * @param array the other ArrayView with a non-const data type
     */
    template<class TT = Type>
    inline ArrayView(const ArrayView<typename std::remove_const<value_type>::type>& array,
                     typename std::enable_if<std::is_const<TT>::value>::type* = 0) :
            _data(array.data()),
            _length(array.size()) {
    }

    /**
     * Creates a wrapper from a vector with a non-const datd type.
     * It is expected that the vector is not resized nor deleted while using
     * this wrapper.
     *
     * @param vector the vector to wrap with a non-const data type
     */
    template<class TT = Type>
    inline ArrayView(const std::vector<typename std::remove_const<value_type>::type>& vector,
                     typename std::enable_if<std::is_const<TT>::value>::type* = 0) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Creates a wrapper from a vector with a non-const data type.
     * It is expected that the vector is not resized nor deleted while using
     * this wrapper.
     *
     * @param vector the vector to wrap with a non-const data type
     */
    template<class TT = Type>
    inline ArrayView(const CppAD::vector<typename std::remove_const<value_type>::type>& vector,
                     typename std::enable_if<std::is_const<TT>::value>::type* = 0) :
            _data(vector.data()),
            _length(vector.size()) {
    }

    /**
     * Creates a wrapper from a std::array with a non-const data type.
     * It is expected that std::array is not deleted while using this wrapper.
     *
     * @param array the std::array to wrap with a non-const data type
     */
    template<std::size_t S, class TT = Type>
    inline ArrayView(const std::array<typename std::remove_const<value_type>::type, S>& array,
                     typename std::enable_if<std::is_const<TT>::value>::type* = 0) :
            _data(array.data()),
            _length(S) {
    }

    /**
     * Creates a wrapper from an std::valarray with a non-const data type.
     * It is expected that std::valarray is not deleted while using this wrapper.
     *
     * @param array the valarray to wrap with a non-const data type
     */
    template<class TT = Type>
    inline ArrayView(const std::valarray<typename std::remove_const<value_type>::type>& array,
                     typename std::enable_if<std::is_const<TT>::value>::type* = 0) :
            _data(array.size() > 0 ? &array[0] : nullptr),
            _length(array.size()) {
    }

    /**
     * Copy constructor
     * @param x
     */
    inline ArrayView(const ArrayView& x) = default;

    /**
     * Desctructor
     */
    virtual ~ArrayView() = default;

    /**
     * @return number of elements in the array.
     */
    inline size_t size() const noexcept {
        return _length;
    }

    /**
     * @return number of elements in the array.
     */
    inline size_type max_size() const noexcept {
        return _length;
    }

    inline bool empty() const noexcept {
        return size() == 0;
    }

    /**
     * @return raw pointer to the array
     */
    inline pointer data() noexcept {
        return _data;
    }

    /**
     * @return raw pointer to the array
     */
    inline const_pointer data() const noexcept {
        return _data;
    }

    inline void fill(const value_type& u) {
        std::fill_n(begin(), size(), u);
    }

    /**
     * @return an ArrayView encapsulating the first n elements
     */
    inline ArrayView<value_type> head(size_t n) {
        CPPADCG_ASSERT_KNOWN(n <= size(), "ArrayView::head() size must be equal to or greater than the array size");
        return ArrayView<value_type> (_data, n);
    }

    /**
     * @return an ArrayView encapsulating the first n elements
     */
    inline ArrayView<const value_type> head(size_t n) const {
        CPPADCG_ASSERT_KNOWN(n <= size(), "ArrayView::head() size must be equal to or greater than the array size");
        return ArrayView<const value_type> (_data, n);
    }

    /**
     * @return an ArrayView encapsulating the last n elements
     */
    inline ArrayView<value_type> tail(size_t n) {
        CPPADCG_ASSERT_KNOWN(n <= size(), "ArrayView::tail() size must be equal to or greater than the array size");
        return ArrayView<value_type> (_data + (size() - n), n);
    }

    /**
     * @return an ArrayView encapsulating the last n elements
     */
    inline ArrayView<const value_type> tail(size_t n) const {
        CPPADCG_ASSERT_KNOWN(n <= size(), "ArrayView::tail() size must be equal to or greater than the array size");
        return ArrayView<const value_type> (_data + (size() - n), n);
    }

    /**
     * @return an ArrayView encapsulating n elements starting at position start
     */
    inline ArrayView<value_type> segment(size_t start,
                                         size_t n) {
        CPPADCG_ASSERT_KNOWN(start < size(), "ArrayView::segment() start index must be lower than the array size");
        CPPADCG_ASSERT_KNOWN(start + n <= size(), "ArrayView::segment() the new segment will end after the end of this array");
        return ArrayView<value_type> (_data + start, n);
    }

    /**
     * @return an ArrayView encapsulating n elements starting at position start
     */
    inline ArrayView<const value_type> segment(size_t start,
                                               size_t n) const {
        CPPADCG_ASSERT_KNOWN(start < size(), "ArrayView::segment() start index must be lower than the array size");
        CPPADCG_ASSERT_KNOWN(start + n <= size(), "ArrayView::segment() the new segment will end after the end of this array");
        return ArrayView<const value_type> (_data + start, n);
    }

    inline void swap(ArrayView& other) noexcept {
        std::swap(other._data, _data);
        std::swap(other._length, _length);
    }

    // Iterators.
    inline iterator begin() noexcept {
        return iterator(data());
    }

    inline const_iterator begin() const noexcept {
        return const_iterator(data());
    }

    inline iterator end() noexcept {
        return iterator(data() + size());
    }

    inline const_iterator end() const noexcept {
        return const_iterator(data() + size());
    }

    inline reverse_iterator rbegin() noexcept {
        return reverse_iterator(end());
    }

    inline const_reverse_iterator rbegin() const noexcept {
        return const_reverse_iterator(end());
    }

    inline reverse_iterator rend() noexcept {
        return reverse_iterator(begin());
    }

    inline const_reverse_iterator rend() const noexcept {
        return const_reverse_iterator(begin());
    }

    inline const_iterator cbegin() const noexcept {
        return const_iterator(data());
    }

    inline const_iterator cend() const noexcept {
        return const_iterator(data() + size());
    }

    inline const_reverse_iterator crbegin() const noexcept {
        return const_reverse_iterator(end());
    }

    inline const_reverse_iterator crend() const noexcept {
        return const_reverse_iterator(begin());
    }

    // Element access.
    inline reference operator[](size_type i) {
        CPPADCG_ASSERT_KNOWN(i < size(), "ArrayView::operator[] index is equal to or greater than the array size");
        return _data[i];
    }

    inline const_reference operator[](size_type i) const {
        CPPADCG_ASSERT_KNOWN(i < size(), "ArrayView::operator[] index is equal to or greater than the array size");
        return _data[i];
    }

    inline reference at(size_type i) {
        if (i >= size())
            throw CGException("ArrayView::at() index ", i, " is equal to or greater than the array size ", size());
        return _data[i];
    }

    inline const_reference at(size_type i) const {
        if (i >= size())
            throw CGException("ArrayView::at() index ", i, " is equal to or greater than the array size ", size());
        return _data[i];
    }

    inline reference front() {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayView: cannot call front for an empty array");
        return *begin();
    }

    inline const_reference front() const {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayView: cannot call front for an empty array");
        return _data[0];
    }

    inline reference back() {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayView: cannot call back for an empty array");
        return *(end() - 1);
    }

    inline const_reference back() const {
        CPPADCG_ASSERT_KNOWN(!empty(), "ArrayView: cannot call back for an empty array");
        return _data[size() - 1];
    }

public:
    inline ArrayView& operator=(const ArrayView& x) {
        if (&x == this)
            return *this;

        if (x.size() != size())
            throw CGException("ArrayView: assigning an array with different size: the left hand side array has the size ", size(),
                              " while the right hand side array has the size ", x.size(), ".");

        for(size_t i = 0; i < _length; ++i) {
            _data[i] = x._data[i];
        }

        return *this;
    }

    template < typename TT = const Type,
               typename = typename std::enable_if<!std::is_same<Type, TT>::value && std::is_assignable<Type&, TT&>::value>::type >
    inline ArrayView& operator=(const ArrayView<TT>& x) {
        if (x.size() != size())
            throw CGException("ArrayView: assigning an array with different size: the left hand side array has the size ", size(),
                              " while the right hand side array has the size ", x.size(), ".");

        const auto* dd = x.data();
        for (size_t i = 0; i < _length; ++i) {
            _data[i] = dd[i];
        }

        return *this;
    }

    inline ArrayView& operator=(const std::vector<Type>& x) {
        if (x.size() != size())
            throw CGException("ArrayView: assigning an array with different size: the left hand side array has the size ", size(),
                              " while the right hand side array has the size ", x.size(), ".");

        for(size_t i = 0; i < _length; ++i) {
            _data[i] = x[i];
        }

        return *this;
    }

    inline ArrayView& operator=(const CppAD::vector<Type>& x) {
        if (x.size() != size())
            throw CGException("ArrayView: assigning an array with different size: the left hand side array has the size ", size(),
                              " while the right hand side array has the size ", x.size(), ".");

        for(size_t i = 0; i < _length; ++i) {
            _data[i] = x[i];
        }

        return *this;
    }

};

/**
 * ArrayView output.
 *
 * @param os  stream to write the vector to
 * @param array  array that is output
 * @return the original stream
 */
template<class Type>
inline std::ostream& operator<<(std::ostream& os,
                                const ArrayView<Type>& array) {
    size_t i = 0;
    size_t n = array.size();

    os << "{ ";
    while (i < n) {
        os << array[i++];
        if (i < n)
            os << ", ";
    }
    os << " }";
    return os;
}

} // END cg namespace
} // END CppAD namespace

# endif
