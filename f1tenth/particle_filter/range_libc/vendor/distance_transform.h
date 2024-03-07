// Copyright (c) 2016 Giorgio Marcias
//
// This file is part of distance_transform, a C++11 implementation of the
// algorithm in "Distance Transforms of Sampled Functions"
// Pedro F. Felzenszwalb, Daniel P. Huttenlocher
// Theory of Computing, Vol. 8, No. 19, September 2012
//
// This source code is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com

#ifndef distance_transform_h
#define distance_transform_h

#include <cstddef>
#include <cstring>
#include <memory>
#include <type_traits>
#include <exception>
#include <sstream>
#include <utility>
#include <limits>

namespace dt {

/// The MArray class represents a D-dimensional dope vector (https://en.wikipedia.org/wiki/Dope_vector) of scalar type T.
/// Given an array stored sequentially in memory, this class wraps it provinding a multi-dimensional matrix interface.
/// It is possible to take slices, windows or even permutations without actually modifying the underlying memory, no element
/// hence moved.
template < typename T, std::size_t D >
class MArray {
public:
    /**
     *    @brief Default constructor.
     */
    MArray() : _array(nullptr), _accumulatedOffset(0)
    {
        for (std::size_t i = 0; i < D; ++i) {
            _size[i] = 0;
            _offset[i] = 0;
        }
    }

    /**
     *    @brief Initializer contructor.
     *    @param array              Pointer to (part of) an array in memory to wrap.
     *    @param accumulatedOffset  Offset from the origin of the array.
     *    @param size               Sizes of the D-dimensional matrix.
     */
    MArray(const T *array, const std::size_t accumulatedOffset, const std::size_t size[D])
        : _array(const_cast<T*>(array))
        , _accumulatedOffset(_array ? accumulatedOffset : 0)
    {
        for (std::size_t i = 0; i < D; ++i)
            _size[i] = _array ? size[i] : 0;
        _offset[D-1] = 1;
        for (std::size_t j = D-1; j > 0; --j)
            _offset[j-1] = _size[j] * _offset[j];
    }

    /**
     *    @brief Initializer contructor.
     *    @param array              Pointer to (part of) an array in memory to wrap.
     *    @param accumulatedOffset  Offset from the origin of the array.
     *    @param size               Sizes of the D-dimensional matrix.
     *    @param offset             Offsets in each dimension, i.e. jumps in memory.
     */
    MArray(const T *array, const std::size_t accumulatedOffset, const std::size_t size[D], const std::size_t offset[D])
        : _array(const_cast<T*>(array))
        , _accumulatedOffset(_array ? accumulatedOffset : 0)
    {
        for (std::size_t i = 0; i < D; ++i) {
            _size[i] = _array ? size[i] : 0;
            _offset[i] = _array ? offset[i] : 0;
        }
    }

    /**
     *    @brief Copy constructor.
     */
    MArray(const MArray &ma)
        : _array(ma._array)
        , _accumulatedOffset(ma._accumulatedOffset)
    {
        for (std::size_t i = 0; i < D; ++i) {
            _size[i] = ma._size[i];
            _offset[i] = ma._offset[i];
        }
    }

    /**
     *    @brief Move constructor.
     */
    MArray(MArray &&ma)
        : _array(ma._array)
        , _accumulatedOffset(ma._accumulatedOffset)
    {
        ma._array = nullptr;
        for (std::size_t i = 0; i < D; ++i) {
            _size[i] = ma._size[i];
            ma._size[i] = 0;
            _offset[i] = ma._offset[i];
            ma._offset[i] = 0;
        }
    }

    /**
     *    @brief Copy assignment operator.
     */
    inline MArray & operator=(const MArray &ma)
    {
        if (&ma != this) {
            _array = ma._array;
            _accumulatedOffset = ma._accumulatedOffset;
            for (std::size_t i = 0; i < D; ++i) {
                _size[i] = ma._size[i];
                _offset[i] = ma._offset[i];
            }
        }
        return *this;
    }

    /**
     *    @brief Move assignment operator.
     */
    inline MArray & operator=(MArray &&ma)
    {
        if (&ma != this) {
            _array = ma._array;
            ma._array = nullptr;
            _accumulatedOffset = ma._accumulatedOffset;
            ma._accumulatedOffset = 0;
            for (std::size_t i = 0; i < D; ++i) {
                _size[i] = ma._size[i];
                ma._size[i] = 0;
                _offset[i] = ma._offset[i];
                ma._offset[i] = 0;
            }
        }
        return *this;
    }

    /**
     *    @brief Gives access to the i-th sub-matrix in the first dimension, i.e. m[i][*]...[*].
     *    @param i                  The i-th "row" of this matrix.
     *    @param s                  The output sub-matrix at i.
     */
    inline void at(const std::size_t i, MArray<T, D-1> &s) const
    {
        if (i >= _size[0]) {
            std::stringstream stream;
            stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
            throw std::out_of_range(stream.str());
        }
        s._array = _array + _offset[0] * i;
        s._accumulatedOffset = accumulatedOffset(i);
        for (std::size_t j = 1; j < D; ++j) {
            s._size[j-1] = _size[j];
            s._offset[j-1] = _offset[j];
        }
    }

    /**
     *    @brief Gives access to the i-th sub-matrix in the first dimension, i.e. m[i][*]...[*].
     *    @param i                  The i-th "row" of this matrix.
     *    @return The output sub-matrix at i.
     */
    inline MArray<T, D-1> operator[](const std::size_t i) const
    {
        MArray<T, D-1> s;
        at(i, s);
        return s;
    }

    /**
     *    @brief Gives access to the i-th sub-matrix in the d-th dimension, i.e. m[*]...[i]...[*].
     *    @param d                  The dimension where to slice.
     *    @param i                  The i-th "row" of this matrix.
     *    @param s                  The output sub-matrix at i in the d dimension.
     */
    inline void slice(const std::size_t d, const std::size_t i, MArray<T, D-1> &s) const
    {
        if (d >= D) {
            std::stringstream stream;
            stream << "Index " << d << " is out of range [0, " << D-1 << ']';
            throw std::out_of_range(stream.str());
        }
        if (i >= _size[d]) {
            std::stringstream stream;
            stream << "Index " << i << " is out of range [0, " << _size[d]-1 << ']';
            throw std::out_of_range(stream.str());
        }
        s._array = _array + _offset[d] * i;
        s._accumulatedOffset = accumulatedOffset(i, d);
        std::size_t k = 0;
        for (std::size_t j = 0; j < d; ++j, ++k) {
            s._size[k] = _size[j];
            s._offset[k] = _offset[j];
        }
        for (std::size_t j = d+1; j < D; ++j, ++k) {
            s._size[k] = _size[j];
            s._offset[k] = _offset[j];
        }
    }

    /**
     *    @brief Gives access to the i-th sub-matrix in the d-th dimension, i.e. m[*]...[i]...[*].
     *    @param d                  The dimension where to slice.
     *    @param i                  The i-th "row" of this matrix.
     *    @return The output sub-matrix at i in the d dimension.
     */
    inline MArray<T, D-1> slice(const std::size_t d, const std::size_t i) const
    {
        MArray<T, D-1> s;
        slice(d, i, s);
        return s;
    }

    /**
     *    @brief Reorders the sub-matrixes s.t. the one at 0 <= i < D goes to 0 <= order[i] < D, i.e. m[*]..[*]_i...[*] swaps with m[*]...[i]...[*]_order[i]...[*].
     *    @param order              A permutation of the matrix indices.
     *    @param p                  The output permuted matrix.
     *    @note Example: transpose a 2D matrix by swapping the access indices - MArray<T,2> m, mt; std::size_t trans_ord[2] = {1, 0}; m.permute(trans_ord, mt);
     */
    inline void permute(const std::size_t order[D], MArray<T, D> &p)
    {
        bool included[D];
        for (std::size_t d = 0; d < D; ++d)
            included[d] = false;
        for (std::size_t d = 0; d < D; ++d) {
            if (order[d] >= D) {
                std::stringstream stream;
                stream << "Index " << order[d] << " is out of range [0, " << D-1 << ']';
                throw std::out_of_range(stream.str());
            }
            if (included[order[d]]) {
                std::stringstream stream;
                stream << "Dimension " << order[d] << " duplicated.";
                throw std::invalid_argument(stream.str());
            }
            included[order[d]] = true;
        }
        p._array = _array;
        p._accumulatedOffset = _accumulatedOffset;
        for (std::size_t d = 0; d < D; ++d) {
            p._size[d] = _size[order[d]];
            p._offset[d] = _offset[order[d]];
        }
    }

    /**
     *    @brief Reorders the sub-matrixes s.t. the one at 0 <= i < D goes to 0 <= order[i] < D, i.e. m[*]..[*]_i...[*] swaps with m[*]...[i]...[*]_order[i]...[*].
     *    @param order              A permutation of the matrix indices.
     *    @return The output permuted matrix.
     *    @note Example: transpose a 2D matrix by swapping the access indices - MArray<T,2> m, mt; std::size_t trans_ord[2] = {1, 0}; mt = m.permute(trans_ord);
     */
    inline MArray<T, D> permute(const std::size_t order[D])
    {
        MArray<T, D> p;
        permute(order, p);
        return p;
    }

    /**
     *    @brief Extracts a D-dimensional window from this matrix.
     *    @param start              The initial offset of the window in each dimension.
     *    @param size               The sizes of the window.
     *    @param p                  The output sub-matrix.
     */
    inline void window(const std::size_t start[D], const std::size_t size[D], MArray<T, D> &p)
    {
        for (std::size_t d = 0; d < D; ++d) {
            if (start[d] >= _size[d]) {
                std::stringstream stream;
                stream << "Index " << start[d] << " is out of range [0, " << _size[d] << ']';
                throw std::out_of_range(stream.str());
            }
            if (start[d] + size[d] > _size[d]) {
                std::stringstream stream;
                stream << "Window size " << size[d] << " is out of range [" << 0 << ", " << _size[d] - start[d] << ']';
                throw std::out_of_range(stream.str());
            }
        }
        p._accumulatedOffset = _accumulatedOffset;
        for (std::size_t d = 0; d < D; ++d)
            p._accumulatedOffset += _offset[d] * start[d];
        p._array = _array + (p._accumulatedOffset - _accumulatedOffset);
        for (std::size_t d = 0; d < D; ++d) {
            p._size[d] = size[d];
            p._offset[d] = _offset[d];
        }
    }

    /**
     *    @brief Extracts a D-dimensional window from this matrix.
     *    @param start              The initial offset of the window in each dimension.
     *    @param size               The sizes of the window.
     *    @return The output sub-matrix.
     */
    inline MArray<T, D> window(const std::size_t start[D], const std::size_t size[D])
    {
        MArray<T, D> w;
        window(start, size, w);
        return w;
    }

    /**
     *    @brief Gives the size of this matrix in the d dimension.
     *    @param d                  The dimension whose size is requested.
     *    @return The size of this matrix at dimension d.
     */
    inline std::size_t size(const std::size_t d = 0) const
    {
        if (d >= D) {
            std::stringstream stream;
            stream << "Index " << d << " is out of range [0, " << D-1 << ']';
            throw std::out_of_range(stream.str());
        }
        return _size[d];
    }

    /**
     *    @brief Gives the sizes of this matrix.
     *    @param s                  The output array that will contain the sizes of this matrix.
     */
    inline void size(std::size_t s[D]) const
    {
        for (std::size_t i = 0; i < D; ++i)
            s[i] = _size[i];
    }

    /**
     *    @brief Gives the total size (number of elements in memory) of this matrix.
     *    @return The total number of elements in memory.
     */
    inline std::size_t totalSize() const
    {
        std::size_t total = _size[0];
        for (std::size_t i = 1; i < D; ++i)
            total *= _size[i];
        return total;
    }

    /**
     *    @brief Gives the total offset, from the beginning of the stored array, of the i-th element at dimension d.
     *    @param i                  The element whose offset is requested.
     *    @param d                  The dimension whose i-th element offset is requested.
     *    @return The total offset from the beggining of the stored array of the i-th element at dimension d.
     */
    inline std::size_t accumulatedOffset(const std::size_t i, const std::size_t d = 0) const
    {
        if (d >= D) {
            std::stringstream stream;
            stream << "Index " << d << " is out of range [0, " << D-1 << ']';
            throw std::out_of_range(stream.str());
        }
        if (i >= _size[d]) {
            std::stringstream stream;
            stream << "Index " << i << " is out of range [0, " << _size[d]-1 << ']';
            throw std::out_of_range(stream.str());
        }
        return  _accumulatedOffset + _offset[d] * i;
    }

private:
    friend class MArray<T, D+1>;
    T                      *_array;                 ///< Pointer in memory to the first element of this matrix.
    std::size_t             _accumulatedOffset;     ///< Offset of the first element of this matrix from the beginning of the stored array.
    std::size_t             _size[D];               ///< Sizes of this matrix, for each dimension.
    std::size_t             _offset[D];             ///< Jumps' offsets from the beginning of a "row" to the beginning of the next one, for each dimension.
};



/// The MArray class represents a 1-dimensional dope vector (https://en.wikipedia.org/wiki/Dope_vector) of scalar type T.
/// Given an array stored sequentially in memory, this class wraps it provinding a mono-dimensional matrix interface.
/// It is possible to take slices, windows or even permutations without actually modifying the underlying memory, no element
/// hence moved.
/// This actually is the basis of the recursive class MArray<T, D> above.
template < typename T >
class MArray<T, 1> {
public:
    /**
     *    @brief Default constructor.
     */
    MArray()
        : _array(nullptr)
        , _accumulatedOffset(0)
    {
        _size[0] = 0;
        _offset[0] = 0;
    }

    /**
     *    @brief Initializer contructor.
     *    @param array              Pointer to (part of) an array in memory to wrap.
     *    @param accumulatedOffset  Offset from the origin of the array.
     *    @param size               Size of the 1-dimensional matrix.
     */
    MArray(const T *array, const std::size_t accumulatedOffset, const std::size_t size)
        : _array(array)
        , _accumulatedOffset(_array ? accumulatedOffset : 0)
    {
        _size[0] = _array ? size : 0;
        _offset[0] = _array ? 1 : 0;
    }

    /**
     *    @brief Initializer contructor.
     *    @param array              Pointer to (part of) an array in memory to wrap.
     *    @param accumulatedOffset  Offset from the origin of the array.
     *    @param size               Size of the 1-dimensional matrix.
     *    @param offset             Offset in memory from one element to the next one.
     */
    MArray(const T *array, const std::size_t accumulatedOffset, const std::size_t size, const std::size_t offset)
        : _array(array)
        , _accumulatedOffset(_array ? accumulatedOffset : 0)
    {
        _size[0] = _array ? size : 0;
        _offset[0] = _array ? offset : 0;
    }

    /**
     *    @brief Initializer contructor.
     *    @param array              Pointer to (part of) an array in memory to wrap.
     *    @param accumulatedOffset  Offset from the origin of the array.
     *    @param size               Size of the 1-dimensional matrix.
     */
    MArray(const T *array, const std::size_t accumulatedOffset, const std::size_t size[1])
        : _array(const_cast<T*>(array))
        , _accumulatedOffset(_array ? accumulatedOffset : 0)
    {
        _size[0] = _array ? size[0] : 0;
        _offset[0] = _array ? 1 : 0;
    }

    /**
     *    @brief Initializer contructor.
     *    @param array              Pointer to (part of) an array in memory to wrap.
     *    @param accumulatedOffset  Offset from the origin of the array.
     *    @param size               Size of the 1-dimensional matrix.
     *    @param offset             Offset in memory from one element to the next one.
     */
    MArray(const T *array, const std::size_t accumulatedOffset, const std::size_t size[1], const std::size_t offset[1])
        : _array(const_cast<T*>(array))
        , _accumulatedOffset(_array ? accumulatedOffset : 0)
    {
        _size[0] = _array ? size[0] : 0;
        _offset[0] = _array ? offset[0] : 0;
    }

    /**
     *    @brief Copy constructor.
     */
    MArray(const MArray &ma)
        : _array(ma._array)
        , _accumulatedOffset(ma._accumulatedOffset)
    {
        _size[0] = ma._size[0];
        _offset[0] = ma._offset[0];
    }

    /**
     *    @brief Move constructor.
     */
    MArray(MArray &&ma)
        : _array(ma._array)
        , _accumulatedOffset(ma._accumulatedOffset)
    {
        _size[0] = ma._size[0];
        _offset[0] = ma._offset[0];
        ma._array = nullptr;
        ma._accumulatedOffset = 0;
        ma._size[0] = 0;
        ma._offset[0] = 0;
    }

    /**
     *    @brief Copy assignment operator.
     */
    MArray & operator=(const MArray &ma)
    {
        if (&ma != this) {
            _array = ma._array;
            _accumulatedOffset = ma._accumulatedOffset;
            _size[0] = ma._size[0];
            _offset[0] = ma._offset[0];
        }
        return *this;
    }

    /**
     *    @brief Move assignment operator.
     */
    MArray & operator=(MArray &&ma)
    {
        if (&ma != this) {
            _array = ma._array;
            _accumulatedOffset = ma._accumulatedOffset;
            _size[0] = ma._size[0];
            _offset[0] = ma._offset[0];
            ma._array = nullptr;
            ma._accumulatedOffset = 0;
            ma._size[0] = 0;
            ma._offset[0] = 0;
        }
        return *this;
    }

    /**
     *    @brief Gives constant access to the i-th element, i.e. m[i].
     *    @param i                  The i-th element of this vector
     *    @return The output element at i.
     */
    inline const T & operator[](const std::size_t i) const
    {
        if (i >= _size[0]) {
            std::stringstream stream;
            stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
            throw std::out_of_range(stream.str());
        }
        return *(_array + i * _offset[0]);
    }

    /**
     *    @brief Gives access to the i-th element, i.e. m[i].
     *    @param i                  The i-th element of this vector
     *    @return The output element at i.
     */
    inline T & operator[](const std::size_t i)
    {
        if (i >= _size[0]) {
            std::stringstream stream;
            stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
            throw std::out_of_range(stream.str());
        }
        return *(_array + i * _offset[0]);
    }

    /**
     *    @brief Gives constant access to the i-th element, i.e. m[i].
     *    @param i                  The i-th element of this vector.
     *    @return The output element at i.
     */
    inline const T & slice(const std::size_t i) const
    {
        return *this[i];
    }

    /**
     *    @brief Gives access to the i-th element, i.e. m[i].
     *    @param i                  The i-th element of this vector.
     *    @return The output element at i.
     */
    inline T & slice(const std::size_t i)
    {
        return *this[i];
    }

    /**
     *    @brief Extracts a 1-dimensional window from this vector.
     *    @param start              The initial offset of the window.
     *    @param size               The size of the window.
     *    @param p                  The output sub-vector.
     */
    inline void window(const std::size_t start[1], const std::size_t size[1], MArray<T, 1> &p)
    {
        if (start[0] >= _size[0]) {
            std::stringstream stream;
            stream << "Index " << start[0] << " is out of range [0, " << _size[0] << ']';
            throw std::out_of_range(stream.str());
        }
        if (start[0] + size[0] > _size[0]) {
            std::stringstream stream;
            stream << "Window size " << size[0] << " is out of range [" << 0 << ", " << _size[0] - start[0] << ']';
            throw std::out_of_range(stream.str());
        }
        p._accumulatedOffset = _accumulatedOffset + _offset[0] * start[0];
        p._array = _array + (p._accumulatedOffset - _accumulatedOffset);
        p._size[0] = size[0];
        p._offset[0] = _offset[0];
    }

    /**
     *    @brief Extracts a 1-dimensional window from this vector.
     *    @param start              The initial offset of the window.
     *    @param size               The size of the window.
     *    @return The output sub-vector.
     */
    inline MArray<T, 1> window(const std::size_t start[1], const std::size_t size[1])
    {
        MArray<T, 1> w;
        window(start, size, w);
        return w;
    }

    /**
     *    @brief Extracts a 1-dimensional window from this vector.
     *    @param start              The initial offset of the window.
     *    @param size               The size of the window.
     *    @param p                  The output sub-vector.
     */
    inline void window(const std::size_t start, const std::size_t size, MArray<T, 1> &p)
    {
        if (start >= _size[0]) {
            std::stringstream stream;
            stream << "Index " << start << " is out of range [0, " << _size[0] << ']';
            throw std::out_of_range(stream.str());
        }
        if (start + size > _size[0]) {
            std::stringstream stream;
            stream << "Window size " << size << " is out of range [" << 0 << ", " << _size[0] - start << ']';
            throw std::out_of_range(stream.str());
        }
        p._accumulatedOffset = _accumulatedOffset + _offset[0] * start;
        p._array = _array + (p._accumulatedOffset - _accumulatedOffset);
        p._size[0] = size;
        p._offset[0] = _offset[0];
    }

    /**
     *    @brief Extracts a 1-dimensional window from this vector.
     *    @param start              The initial offset of the window.
     *    @param size               The size of the window.
     *    @return The output sub-vector.
     */
    inline MArray<T, 1> window(const std::size_t start, const std::size_t size)
    {
        MArray<T, 1> w;
        window(start, size, w);
        return w;
    }

    /**
     *    @brief Gives the size of this vector.
     *    @return The output size of this vector.
     */
    inline std::size_t size() const
    {
        return _size[0];
    }

    /**
     *    @brief Gives the size of this vector.
     *    @param s                  The output size of this vector.
     */
    inline void size(std::size_t s[1]) const
    {
        s[0] = _size[0];
    }

    /**
     *    @brief Gives the total size (number of elements in memory) of this vector.
     *    @return The total number of elements in memory.
     */
    inline std::size_t totalSize() const
    {
        return size();
    }

    /**
     *    @brief Gives the total offset, from the beginning of the stored array, of the i-th element.
     *    @param i                  The element whose offset is requested.
     *    @return The total offset from the beggining of the stored array of the i-th element.
     */
    inline std::size_t accumulatedOffset(const std::size_t i = 0) const
    {
        if (i >= _size[0]) {
            std::stringstream stream;
            stream << "Index " << i << " is out of range [0, " << _size[0]-1 << ']';
            throw std::out_of_range(stream.str());
        }
        return _accumulatedOffset + _offset[0] * i;
    }

private:
    friend class MArray<T, 2>;
    T                      *_array;                 ///< Pointer in memory to the first element of this vector.
    std::size_t             _accumulatedOffset;     ///< Offset of the first element of this vector from the beginning of the stored array.
    std::size_t             _size[1];               ///< Size of this vector.
    std::size_t             _offset[1];             ///< Jumps' offset from an element to the next one.
};


/// The MMArray class is a wrapper of MArray providing a built-in memory storage and management.
template < typename T, std::size_t D >
class MMArray : public MArray<T, D> {
public:
    /**
     *    @brief Default constructor.
     */
    MMArray()
        : MArray<T, D>()
        , _arrayPtr(nullptr)
    { }

    /**
     *    @brief Initializer contructor.
     *    @param size               Sizes of the D-dimensional matrix.
     */
    MMArray(const std::size_t size[D])
        : MArray<T, D>()
        , _arrayPtr(nullptr)
    {
        resize(size);
    }

    /**
     *    @brief Copy constructor.
     */
    MMArray(const MMArray<T, D> &mma)
        : MArray<T, D>()
        , _arrayPtr(nullptr)
    {
        *this = mma;
    }

    /**
     *    @brief Move constructor.
     */
    MMArray(MMArray &&mma)
        : MArray<T, D>(std::forward<MMArray<T, D>>(mma))
        , _arrayPtr(std::move(mma._arrayPtr))
    { }

    /**
     *    @brief Copy assignment operator.
     */
    MMArray & operator=(const MMArray &mma)
    {
        if (&mma != this) {
            std::size_t size[D];
            mma.size(size);
            resize(size);
            std::memcpy(_arrayPtr.get(), mma._arrayPtr.get(), MArray<T, D>::totalSize() * sizeof(T));
        }
        return *this;
    }

    /**
     *    @brief Move assignment operator.
     */
    MMArray & operator=(MMArray &&mma)
    {
        if (&mma != this) {
            MArray<T, D>::operator=(std::forward<MMArray<T, D>>(mma));
            _arrayPtr = std::move(mma._arrayPtr);
        }
        return *this;
    }

    /**
     *    @brief Changes the sizes of this matrix.
     *    @param size               The new sizes.
     */
    inline void resize(const std::size_t size[D])
    {
        std::size_t total = size[0];
        for (std::size_t i = 1; i < D; ++i)
            total *= size[i];
        if (total > 0) {
            if (total != MArray<T, D>::totalSize())
                _arrayPtr.reset(new T[total]);                                  // Be aware: data is LOST!
            MArray<T, D>::operator=(MArray<T, D>(_arrayPtr.get(), 0, size));
        } else {
            clear();
        }
    }

    /**
     *    @brief Empties this array, making its size 0.
     */
    inline void clear()
    {
        _arrayPtr.reset(nullptr);
        MArray<T, D>::operator=(MArray<T, D>());
    }

private:
    std::unique_ptr<T[]>    _arrayPtr;              ///< Sharp pointer in memory to this matrix, with automatic storage (de)allocation.
};

}

// Copyright (c) 2016 Giorgio Marcias
//
// This file is part of distance_transform, a C++11 implementation of the
// algorithm in "Distance Transforms of Sampled Functions"
// Pedro F. Felzenszwalb, Daniel P. Huttenlocher
// Theory of Computing, Vol. 8, No. 19, September 2012
//
// This source code is subject to Apache 2.0 License.
//
// Author: Giorgio Marcias
// email: marcias.giorgio@gmail.com


#include <cmath>

namespace dt {

/// The DistanceTransform class provides static method to compute a distance field over any multi-dimensional regularly sampled function.
/// The dimension is fixed at compile time.
/// It is also possible to compute the index of the nearest minimum of each sample.
class DistanceTransform {
public:
    /**
     *    @brief Compute the L2-norm distance field D of a DIM-dimensional sampled function f. D gets the distance from the local minima of f.
     *    @param f              A DIM-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
     *    @note Arrays f and D can also be the same.
     */
    template < typename Scalar = float, std::size_t DIM = 2 >
    inline static void distanceTransformL2(const MMArray<Scalar, DIM> &f, MMArray<Scalar, DIM> &D, const bool squared = false)
    {
        if (&D != &f) {
            std::size_t size[DIM];
            f.size(size);
            D.resize(size);
        }
        MMArray<Scalar, DIM> fCopy(f);
        MArray<Scalar, DIM> tmpF(fCopy);
        MArray<Scalar, DIM> tmpD(D);
        MArray<Scalar, DIM-1> f_dq;
        MArray<Scalar, DIM-1> D_dq;
        // compute for each slice
        for (std::size_t d = 0; d < DIM; ++d) {
            for (std::size_t q = 0; q < tmpF.size(d); ++q) {
                tmpF.slice(d, q, f_dq);
                tmpD.slice(d, q, D_dq);
                distanceL2(f_dq, D_dq);
            }
            std::swap(tmpD, tmpF);
        }
        if (DIM % 2 == 0)
            D = std::move(fCopy);
        if (!squared)
            element_wiseSquareRoot(D);
    }

    /**
     *    @brief Compute the L2-norm distance field D of a 1-dimensional sampled function f. D gets the distance from the local minima of f.
     *    @param f              A 1-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
     *    @note Arrays f and D can also be the same.
     */
    template < typename Scalar = float >
    inline static void distanceTransformL2(const MMArray<Scalar, 1> &f, MMArray<Scalar, 1> &D, const bool squared = false)
    {
        std::size_t size[1];
        f.size(size);
        if (&D != &f)
            D.resize(size);
        distanceL2(f, D);
        if (!squared) {
            element_wiseSquareRoot(D);
        }
    }

    /**
     *    @brief Compute the L2-norm distance field D of a DIM-dimensional sampled function f. D gets the distance from the local minima of f.
     *           Compute also the (index of the) nearest local minimum of each sample.
     *    @param f              A DIM-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     *    @param I              Resulting array containing the (index of the) local minimum for each sample.
     *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
     *    @note Arrays f and D can also be the same.
     */
    template < typename Scalar = float, std::size_t DIM = 2 >
    inline static void distanceTransformL2(const MMArray<Scalar, DIM> &f, MMArray<Scalar, DIM> &D, MMArray<std::size_t, DIM> &I, const bool squared = false)
    {
        std::size_t size[DIM];
        f.size(size);
        if (&D != &f)
            D.resize(size);
        I.resize(size);
        MMArray<Scalar, DIM> fCopy(f);          // make a safe copy of f
        MArray<Scalar, DIM> tmpF(fCopy);
        MArray<Scalar, DIM> tmpD(D);
        MMArray<std::size_t, DIM> ICopy(I);     // make a safe copy of I
        MArray<std::size_t, DIM> Ipre(ICopy);
        MArray<std::size_t, DIM> Ipost(I);
        MArray<Scalar, DIM-1> f_dq;
        MArray<Scalar, DIM-1> D_dq;
        MArray<std::size_t, DIM-1> Ipre_dq;
        MArray<std::size_t, DIM-1> Ipost_dq;
        // initialize I
        initializeIndices(Ipre);
        // compute for each slice
        for (std::size_t d = 0; d < DIM; ++d) {
            for (std::size_t q = 0; q < tmpF.size(d); ++q) {
                tmpF.slice(d, q, f_dq);
                tmpD.slice(d, q, D_dq);
                Ipre.slice(d, q, Ipre_dq);
                Ipost.slice(d, q, Ipost_dq);
                distanceL2(f_dq, D_dq, Ipre_dq, Ipost_dq);
            }
            std::swap(tmpD, tmpF);
            std::swap(Ipost, Ipre);
        }
        if (DIM % 2 == 0) {
            D = std::move(fCopy);
            I = std::move(ICopy);
        }
        if (!squared)
            element_wiseSquareRoot(D);
    }

    /**
     *    @brief Compute the L2-norm distance field D of a 1-dimensional sampled function f. D gets the distance from the local minima of f.
     *           Compute also the (index of the) nearest local minimum of each sample.
     *    @param f              A 1-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     *    @param I              Resulting array containing the (index of the) local minimum for each sample.
     *    @param squared        Compute squared distances (L2)^2 - avoiding to compute square roots - (true) or keep them normal (false - default).
     *    @note Arrays f and D can also be the same.
     */
    template < typename Scalar = float >
    inline static void distanceTransformL2(const MMArray<Scalar, 1> &f, MMArray<Scalar, 1> &D, MMArray<std::size_t, 1> &I, const bool squared = false)
    {
        std::size_t size[1];
        f.size(size);
        if (&D != &f)
            D.resize(size);
        I.resize(size);
        distanceL2(f, D, I);
        if (!squared) {
            element_wiseSquareRoot(D);
        }
    }

    /**
     *    @brief Set up the initial indices of a DIM-dimensional sampled function.
     *    @param I              Resulting array containing the initial index for each sample.
     */
    template < std::size_t DIM >
    inline static void initializeIndices(MArray<std::size_t, DIM> &I)
    {
        MArray<std::size_t, DIM-1> I_q;
        for (std::size_t q = 0; q < I.size(); ++q) {
            I.at(q, I_q);
            initializeIndices(I_q);
        }
    }

    /**
     *    @brief Set up the initial indices of a 1-dimensional sampled function.
     *    @param I              Resulting array containing the initial index for each sample.
     */
    inline static void initializeIndices(MArray<std::size_t, 1> &I)
    {
        for (std::size_t q = 0; q < I.size(); ++q)
            I[q] = I.accumulatedOffset(q);
    }



private:
    /**
     *    @brief The actual distance field computation is done by recursive calls of this method, in lower dimenional sub-functions.
     *    @param f              A DIM-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     */
    template < typename Scalar = float, std::size_t DIM >
    inline static void distanceL2(const MArray<Scalar, DIM> &f, MArray<Scalar, DIM> &D)
    {
        MArray<Scalar, DIM-1> f_q, D_q;
        // compute distance at lower dimensions for each hyperplane
        for (std::size_t q = 0; q < f.size(); ++q) {
            f.at(q, f_q);
            D.at(q, D_q);
            distanceL2(f_q, D_q);
        }
    }

    /**
     *    @brief The actual distance field computation as in the "Distance Transforms of Sampled Functions" paper, performed in a mono-dimensional function.
     *    @param f              A 1-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     */
    template < typename Scalar = float >
    inline static void distanceL2(const MArray<Scalar, 1> &f, MArray<Scalar, 1> &D)
    {
        if (f.size() == 0 || f.size() > D.size())
            return;
        if (f.size() == 1) {
            D[0] = f[0];
            return;
        }
        std::size_t k = 0;                          // index of rightmost parabola in lower envelope
        std::size_t *v = new std::size_t[f.size()]; // locations of parabolas in lower envelope
        double *z = new double[f.size() + 1];       // locations of boundaries between parabolas
        double s = double(0);
        // initialization
        v[0] = 0;
        z[0] = -std::numeric_limits<double>::max();
        z[1] = std::numeric_limits<double>::max();
        // compute lowest envelope:
        for (std::size_t q = 1; q < f.size(); ++q) {
            ++k;    // this compensates for first line of next do-while block
            do {
                --k;
                // compute horizontal position of intersection between the parabola from q and the current lowest parabola
                s = ((f[q] + q*q) - static_cast<double>(f[v[k]] + v[k]*v[k])) / (2*q - static_cast<double>(2*v[k]));
            } while (s <= z[k]);
            ++k;
            v[k] = q;
            z[k] = s;
            z[k+1] = std::numeric_limits<double>::max();
        }
        // fill in values of distance transform
        k = 0;
        for (std::size_t q = 0; q < f.size(); ++q) {
            while(z[k+1] < static_cast<double>(q))
                ++k;
            D[q] = f[v[k]] + (q - static_cast<Scalar>(v[k]))*(q - static_cast<Scalar>(v[k]));
        }
        // delete allocated memory
        delete[] z;
        delete[] v;
    }

    /**
     *    @brief The actual distance field computation is done by recursive calls of this method, in lower dimenional sub-functions.
     *    @param f              A DIM-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     *    @param I              Resulting array containing the (index of the) local minimum for each sample.
     */
    template < typename Scalar = float, std::size_t DIM >
    inline static void distanceL2(const MArray<Scalar, DIM> &f, MArray<Scalar, DIM> &D, const MArray<std::size_t, DIM> &Ipre, MArray<std::size_t, DIM> &Ipost)
    {
        MArray<Scalar, DIM-1> f_q, D_q;
        MArray<std::size_t, DIM-1> Ipre_q, Ipost_q;
        // compute distance at lower dimensions for each hyperplane
        for (std::size_t q = 0; q < f.size(); ++q) {
            f.at(q, f_q);
            D.at(q, D_q);
            Ipre.at(q, Ipre_q);
            Ipost.at(q, Ipost_q);
            distanceL2(f_q, D_q, Ipre_q, Ipost_q);
        }
    }

    /**
     *    @brief The actual distance field computation as in the "Distance Transforms of Sampled Functions" paper, performed in a mono-dimensional function.
     *    @param f              A 1-dimensional, regularly sampled function.
     *    @param D              The resulting distance field of f.
     *    @param I              Resulting array containing the (index of the) local minimum for each sample.
     */
    template < typename Scalar = float >
    inline static void distanceL2(const MArray<Scalar, 1> &f, MArray<Scalar, 1> &D, const MArray<std::size_t, 1> &Ipre, MArray<std::size_t, 1> &Ipost)
    {
        if (f.size() == 0 || f.size() > D.size())
            return;
        if (f.size() == 1) {
            D[0] = f[0];
            Ipost[0] = Ipre[0];
            return;
        }
        std::size_t k = 0;                          // index of rightmost parabola in lower envelope
        std::size_t *v = new std::size_t[f.size()]; // locations of parabolas in lower envelope
        double *z = new double[f.size() + 1];       // locations of boundaries between parabolas
        double s = double(0);
        // initialization
        v[0] = 0;
        z[0] = -std::numeric_limits<double>::max();
        z[1] = std::numeric_limits<double>::max();
        // compute lowest envelope:
        for (std::size_t q = 1; q < f.size(); ++q) {
            ++k;    // this compensates for first line of next do-while block
            do {
                --k;
                // compute horizontal position of intersection between the parabola from q and the current lowest parabola
                s = ((f[q] + q*q) - static_cast<double>(f[v[k]] + v[k]*v[k])) / (2*q - static_cast<double>(2*v[k]));
            } while (s <= z[k]);
            ++k;
            v[k] = q;
            z[k] = s;
            z[k+1] = std::numeric_limits<double>::max();
        }
        // fill in values of distance transform
        k = 0;
        for (std::size_t q = 0; q < f.size(); ++q) {
            while(z[k+1] < static_cast<double>(q))
                ++k;
            D[q] = f[v[k]] + (q - static_cast<Scalar>(v[k]))*(q - static_cast<Scalar>(v[k]));
            Ipost[q] = Ipre[v[k]];
        }
        // delete allocated memory
        delete[] z;
        delete[] v;
    }

public:
    /**
     *    @brief Compute the square root of each individual element of a DIM-dimensional array.
     *    @param m              A DIM-dimensioanl array whose element have to be square rooted.
     */
    template < typename Scalar = float, std::size_t DIM >
    inline static void element_wiseSquareRoot(MArray<Scalar, DIM> &m)
    {
        MArray<Scalar, DIM-1> mm;
        for (std::size_t q = 0; q < m.size(); ++q) {
            m.at(q, mm);
            element_wiseSquareRoot(mm);
        }
    }

    /**
     *    @brief Compute the square root of each individual element of a 1-dimensional array.
     *    @param m              A 1-dimensioanl array whose element have to be square rooted.
     */
    template < typename Scalar = float >
    inline static void element_wiseSquareRoot(MArray<Scalar, 1> &m)
    {
        for (std::size_t q = 0; q < m.size(); ++q)
            m[q] = static_cast<Scalar>(std::sqrt(m[q]));
    }
};

}

#endif /* distance_transform_h */