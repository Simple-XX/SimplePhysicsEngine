/**
 * @file dtkVector.h
 * @brief dtkVector 头文件
 * @author Chance.H (chancewithchance@outlook.com)
 * @version 1.0
 * @date 2024-7-29
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 */
#ifndef SIMPLEPHYSICSENGINE_DTKVECTOR_H
#define SIMPLEPHYSICSENGINE_DTKVECTOR_H
#include <cassert>
#include <vector>

#include <glm/glm.hpp>

namespace dtk {
    /**
     * @class <dtkVector>
     * @brief 向量
     * @author <>
     * @note
     * 实现任意维的向量
     */
    template <class T, class VectorT = std::vector<T>> struct dtkVector {
        // STL-friendly typedefs

        typedef typename VectorT::iterator iterator;
        typedef typename VectorT::const_iterator const_iterator;
        typedef typename VectorT::size_type size_type;
        typedef long difference_type;
        typedef T& reference;
        typedef const T& const_reference;
        typedef T value_type;
        typedef T* pointer;
        typedef const T* const_pointer;
        typedef typename VectorT::reverse_iterator reverse_iterator;
        typedef typename VectorT::const_reverse_iterator const_reverse_iterator;

        // the actual representation

        unsigned int n;
        VectorT v;

        // the interface

        dtkVector(void) : n(0) {}

        dtkVector(int n_) : n(n_), v(n_) {
            assert(n_ >= 0);
        }

        dtkVector(int n_, VectorT& v_) : n(n_), v(v_) {
            assert(n_ >= 0);
        }

        dtkVector(int n_, const T& value)
            : n(n_), v(n_, value) {
            assert(n_ >= 0);
        }

        dtkVector(int n_, const T& value, size_type max_n_)
            : n(n_), v(n_, value, max_n_) {
            assert(n_ >= 0);
        }

        void init(const T* values) {
            for (unsigned int i = 0; i < n; i++)
                v[i] = values[i];
        }

        template <class OtherVectorT>
        dtkVector(dtkVector<T, OtherVectorT>& other)
            : n(other.n), v(other.v) {}

        ~dtkVector(void) {
#ifndef NDEBUG
            n = 0;
#endif
        }

        const T& operator()(unsigned int i) const {
            assert(i >= 0 && i < n);
            return v[i];
        }

        T& operator()(unsigned int i) {
            assert(i >= 0 && i < n);
            return v[i];
        }

        const T& operator[](unsigned int i) const {
            assert(i >= 0 && i < n);
            return v[i];
        }

        T& operator[](unsigned int i) {
            assert(i >= 0 && i < n);
            return v[i];
        }

        bool operator==(const dtkVector<T>& x) const {
            return n == x.n && v == x.v;
        }

        bool operator!=(const dtkVector<T>& x) const {
            return n != x.n || v != x.v;
        }

        bool operator<(const dtkVector<T>& x) const {
            if (n < x.n)
                return true;
            else if (n > x.n)
                return false;
            return v < x.v;
        }

        bool operator>(const dtkVector<T>& x) const {
            if (n > x.n)
                return true;
            else if (n < x.n)
                return false;
            return v > x.v;
        }

        bool operator<=(const dtkVector<T>& x) const {
            if (n < x.n)
                return true;
            else if (n > x.n)
                return false;
            return v <= x.v;
        }

        bool operator>=(const dtkVector<T>& x) const {
            if (n > x.n)
                return true;
            else if (n < x.n)
                return false;
            return v >= x.v;
        }

        void assign(const T& value) { v.assign(value); }

        void assign(int n_, const T& value) {
            v.assign(n_, value);
            n = n_;
        }

        void assign(int n_, const T* copydata) {
            v.assign(n_, copydata);
            n = n_;
        }

        const T& at(int i) const {
            assert(i >= 0 && i < n);
            return v[i];
        }

        T& at(int i) {
            assert(i >= 0 && i < n);
            return v[i];
        }

        const T& back(void) const {
            assert(v.size());
            return v.back();
        }

        T& back(void) {
            assert(v.size());
            return v.back();
        }

        const_iterator begin(void) const { return v.begin(); }

        iterator begin(void) { return v.begin(); }

        size_type capacity(void) const { return v.capacity(); }

        void clear(void) {
            v.clear();
            n = 0;
        }

        bool empty(void) const { return v.empty(); }

        const_iterator end(void) const { return v.end(); }

        iterator end(void) { return v.end(); }

        void fill(int n_, const T& value) {
            v.fill(n_, value);
            n = n_;
        }

        const T& front(void) const {
            assert(v.size());
            return v.front();
        }

        T& front(void) {
            assert(v.size());
            return v.front();
        }

        size_type max_size(void) const { return v.max_size(); }

        reverse_iterator rbegin(void) { return reverse_iterator(end()); }

        const_reverse_iterator rbegin(void) const {
            return const_reverse_iterator(end());
        }

        reverse_iterator rend(void) { return reverse_iterator(begin()); }

        const_reverse_iterator rend(void) const {
            return const_reverse_iterator(begin());
        }

        void reserve(int reserve_n) {
            v.reserve(reserve_n);
        }

        void resize(int n_) {
            assert(n_ >= 0);
            v.resize(n_);
            n = n_;
        }

        void resize(int n_, const T& value) {
            assert(n_ >= 0);
            v.resize(n_, value);
            n = n_;
        }

        void set_zero(void) { v.set_zero(); }

        size_type size(void) const { return v.size(); }

        void swap(dtkVector<T>& x) {
            std::swap(n, x.n);
            v.swap(x.v);
        }

        void trim(void) { v.trim(); }
    };

    // some common vectors
    typedef dtkVector<double, std::vector<double>> dtkVectorDouble;
    typedef dtkVector<float, std::vector<float>> dtkVectorFloat;
    typedef dtkVector<long long, std::vector<long long>> dtkVectorLLong;
    typedef dtkVector<unsigned long long, std::vector<unsigned long long>> dtkVectorULLong;
    typedef dtkVector<int, std::vector<int>> dtkVectorInt;
    typedef dtkVector<unsigned int, std::vector<unsigned int>> dtkVectorUInt;
    typedef dtkVector<short, std::vector<short>> dtkVectorShort;
    typedef dtkVector<unsigned short, std::vector<unsigned short>> dtkVectorUShort;
    typedef dtkVector<char, std::vector<char>> dtkVectorChar;
    typedef dtkVector<unsigned char, std::vector<unsigned char>> dtkVectorUChar;

    typedef glm::vec2 dtkVector2;
    typedef glm::vec3 dtkVector3;
    typedef glm::vec4 dtkVector4;

}
#endif // SIMPLEPHYSICSENGINE_DTKVECTOR_H