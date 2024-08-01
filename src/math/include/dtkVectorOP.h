/**
 * @file dtkVectorOP.h
 * @brief dtkVectorOP 头文件
 * @author Chance.H (chancewithchance@outlook.com)
 * @version 1.0
 * @date 2024-7-29
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 */
#ifndef SIMPLEPHYSICSENGINE_DTKVECTOROP_H
#define SIMPLEPHYSICSENGINE_DTKVECTOROP_H
#include <cassert>
#include <cmath>

#include "dtkVector.h"
#include "dtkMatrix.h"

namespace dtk {
    template <class T> inline dtkVector<T> transform_identity_vector() {
        dtkVector<T> v(4, 0);
        v[0] = 1;
        v[1] = 1;
        v[2] = 1;
        v[3] = 1;
        return v;
    }

    template <class T> inline dtkVector<T> rotate_x_vector(dtkVector<T> v, T angle) {
        dtkMatrix<T> m = rotate_x_matrix(angle);
        return m * v;
    }

    template <class T> inline dtkVector<T> rotate_y_vector(dtkVector<T> v, T angle) {
        dtkMatrix<T> m = rotate_y_matrix(angle);
        return m * v;
    }

    template <class T> inline dtkVector<T> rotate_z_vector(dtkVector<T> v, T angle) {
        dtkMatrix<T> m = rotate_z_matrix(angle);
        return m * v;
    }

    template <class T> inline dtkVector<T> translate_vector(dtkVector<T> v, T x, T y, T z) {
        dtkMatrix<T> m = translate_matrix(x, y, z);
        return m * v;
    }

    template <class T> inline dtkVector<T> scale_vector(dtkVector<T> v, T x, T y, T z) {
        dtkMatrix<T> m = scale_matrix(x, y, z);
        return m * v;
    }

    template <class T>
    inline dtkVector<T> operator+(const dtkVector<T>& lhs, const dtkVector<T>& rhs) {
        assert(lhs.n == rhs.n);
        dtkVector<T> v(lhs.n);
        for (unsigned int i = 0; i < v.n; i++)
            v[i] = lhs[i] + rhs[i];
        return v;
    }

    template <class T>
    inline dtkVector<T> operator-(const dtkVector<T>& lhs, const dtkVector<T>& rhs) {
        assert(lhs.n == rhs.n);
        dtkVector<T> v(lhs.n);
        for (unsigned int i = 0; i < v.n; i++)
            v[i] = lhs[i] - rhs[i];
        return v;
    }

    template <class T>
    inline dtkVector<T> operator*(T lhs, const dtkVector<T>& rhs) {
        dtkVector<T> v(rhs.n);
        for (unsigned int i = 0; i < v.n; i++)
            v[i] = lhs * rhs[i];
        return v;
    }

    template <class T>
    inline dtkVector<T> operator*(const dtkVector<T>& lhs, T rhs) {
        dtkVector<T> v(lhs.n);
        for (unsigned int i = 0; i < v.n; i++)
            v[i] = lhs[i] * rhs;
        return v;
    }

    template <class T>
    inline T dot(const dtkVector<T>& lhs, const dtkVector<T>& rhs) {
        assert(lhs.n == rhs.n);
        T result = 0;
        for (unsigned int i = 0; i < lhs.n; i++)
            result += lhs[i] * rhs[i];
        return result;
    }

    template <class T>
    inline dtkVector<T> cross(const dtkVector<T>& lhs, const dtkVector<T>& rhs) {
        assert(lhs.n == 3 && rhs.n == 3);
        dtkVector<T> v(3);
        v[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1];
        v[1] = lhs[2] * rhs[0] - lhs[0] * rhs[2];
        v[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
        return v;
    }

    template <class T>
    inline dtkVector<T> normalize(const dtkVector<T>& v) {
        T length = std::sqrt(dot(v, v));
        assert(length != 0);
        return (1 / length) * v;
    }

    template <class T> inline dtkMatrix<T> outer_product(const dtkVector<T>& lhs, const dtkVector<T>& rhs) {
        dtkMatrix<T> m(lhs.n, rhs.n);
        for (unsigned int i = 0; i < lhs.n; i++)
            for (unsigned int j = 0; j < rhs.n; j++)
                m(i, j) = lhs[i] * rhs[j];
        return m;
    }

    template <class T>
    inline dtkVector<T> operator*(const dtkMatrix<T>& lhs, const dtkVector<T>& rhs) {
        assert(lhs.nj == rhs.n);
        dtkVector<T> v(lhs.ni, 0);
        for (unsigned int i = 0; i < lhs.ni; i++)
            for (unsigned int j = 0; j < lhs.nj; j++)
                v[i] += lhs(i, j) * rhs[j];
        return v;
    }

    template <class T>
    inline dtkVector<T> operator*(const dtkVector<T>& lhs, const dtkMatrix<T>& rhs) {
        assert(lhs.n == rhs.ni);
        dtkVector<T> v(rhs.nj, 0);
        for (unsigned int i = 0; i < rhs.nj; i++)
            for (unsigned int j = 0; j < lhs.n; j++)
                v[i] += lhs[j] * rhs(j, i);
        return v;
    }

}
#endif //SIMPLEPHYSICSENGINE_DTKVECTOROP_H