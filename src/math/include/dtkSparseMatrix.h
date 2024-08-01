/**
 * @file dtkSparseMatrix.h
 * @brief dtkSparseMatrix 头文件
 * @author Chance.H (chancewithchance@outlook.com)
 * @version 1.0
 * @date 2024-7-30
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 */
#ifndef SIMPLEPHYSICSENGINE_DTKSPARSEMATRIX_H
#define SIMPLEPHYSICSENGINE_DTKSPARSEMATRIX_H
#include <vector>
#include <cassert>
#include <algorithm>

namespace dtk {

    /**
     * @class <dtkSparseMatrix>
     * @brief 稀疏矩阵
     * @author <>
     * @note
     * 实现稀疏矩阵
     */
    template <class T>
    class dtkSparseMatrix {
    public:
        typedef unsigned int Index;
        typedef T ValueType;

        struct Triplet {
            Index row, col;
            ValueType value;

            Triplet(Index r, Index c, const ValueType& v) : row(r), col(c), value(v) {}
        };

        dtkSparseMatrix() : rows_(0), cols_(0) {}

        dtkSparseMatrix(Index rows, Index cols) : rows_(rows), cols_(cols) {}

        void resize(Index rows, Index cols) {
            rows_ = rows;
            cols_ = cols;
            triplets_.clear();
        }

        void insert(Index row, Index col, const ValueType& value) {
            assert(row < rows_ && col < cols_);
            triplets_.emplace_back(row, col, value);
        }

        void setFromTriplets(const std::vector<Triplet>& triplets) {
            triplets_ = triplets;
        }

        void reserve(Index reserveSize) {
            triplets_.reserve(reserveSize);
        }

        void clear() {
            triplets_.clear();
        }

        Index rows() const { return rows_; }
        Index cols() const { return cols_; }
        Index nonZeros() const { return triplets_.size(); }

        const std::vector<Triplet>& triplets() const { return triplets_; }

        dtkSparseMatrix<T> transpose() const {
            dtkSparseMatrix<T> result(cols_, rows_);
            for (const auto& triplet : triplets_) {
                result.insert(triplet.col, triplet.row, triplet.value);
            }
            return result;
        }

        template <class VectorT>
        std::vector<T> operator*(const VectorT& rhs) const {
            assert(rhs.size() == cols_);
            std::vector<T> result(rows_, T(0));
            for (const auto& triplet : triplets_) {
                result[triplet.row] += triplet.value * rhs[triplet.col];
            }
            return result;
        }

        dtkSparseMatrix<T> operator+(const dtkSparseMatrix<T>& rhs) const {
            assert(rows_ == rhs.rows_ && cols_ == rhs.cols_);
            dtkSparseMatrix<T> result(rows_, cols_);
            result.triplets_ = triplets_;
            for (const auto& triplet : rhs.triplets_) {
                auto it = std::find_if(result.triplets_.begin(), result.triplets_.end(),
                                       [&](const Triplet& t) { return t.row == triplet.row && t.col == triplet.col; });
                if (it != result.triplets_.end()) {
                    it->value += triplet.value;
                }
                else {
                    result.triplets_.emplace_back(triplet.row, triplet.col, triplet.value);
                }
            }
            return result;
        }

        dtkSparseMatrix<T> operator-(const dtkSparseMatrix<T>& rhs) const {
            assert(rows_ == rhs.rows_ && cols_ == rhs.cols_);
            dtkSparseMatrix<T> result(rows_, cols_);
            result.triplets_ = triplets_;
            for (const auto& triplet : rhs.triplets_) {
                auto it = std::find_if(result.triplets_.begin(), result.triplets_.end(),
                                       [&](const Triplet& t) { return t.row == triplet.row && t.col == triplet.col; });
                if (it != result.triplets_.end()) {
                    it->value -= triplet.value;
                }
                else {
                    result.triplets_.emplace_back(triplet.row, triplet.col, -triplet.value);
                }
            }
            return result;
        }

    private:
        Index rows_, cols_;
        std::vector<Triplet> triplets_;
    };

} // namespace dtk
#endif /* SIMPLEPHYSICSENGINE_DTKSPARSEMATRIX_H */