#ifndef SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGSOLVER_H
#define SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGSOLVER_H
#include "dtkPhysMassPoint.h"
#include "dtkPhysMassSpring.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <random>

#ifdef DTK_CUDA
#include <cuda_runtime.h>
#include <cusparse_v2.h>
#include "cusparse_cholesky_solver.h"
#include "step.cuh"
#endif

namespace dtk {
    class dtkPhysMassSpringSolver {
    private:
        typedef Eigen::Vector3f Vector3f;
        typedef Eigen::VectorXf VectorXf;
        typedef Eigen::SparseMatrix<float, Eigen::StorageOptions::RowMajor> SparseMatrixCSR;
        typedef Eigen::SimplicialLLT<SparseMatrixCSR> Cholesky;
        typedef Eigen::Map<Eigen::VectorXf> Map;
        typedef std::pair<unsigned int, unsigned int> Edge;
        typedef Eigen::Triplet<float> Triplet;
        typedef std::vector<Triplet> TripletList;
#ifdef DTK_CUDA
        typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorR;
        typedef Eigen::AMDOrdering<SparseMatrixCSR::StorageIndex> Ordering;
        typedef Ordering::PermutationType PermutationMatrix;
#endif
    public:
        typedef std::shared_ptr<dtkPhysMassSpringSolver> Ptr;

        static dtkPhysMassSpringSolver::Ptr New() {
            return Ptr(new dtkPhysMassSpringSolver());
        }
        static dtkPhysMassSpringSolver::Ptr New(const dtkPhysMassSpring::Ptr& massSpring) {
            return Ptr(new dtkPhysMassSpringSolver(massSpring));
        }

        void solve(unsigned int iter_num);
        enum ClothDropType {
            Sphere,
            Hang
        };

        void satisfy(ClothDropType type = Sphere);
        VectorXf getCurrentState() const { return _current_state; };

#ifdef DTK_CUDA
        static VectorXf cusparse_multiply(const SparseMatrixCSR& J, const VectorXf& x) {
            int rows = J.rows();
            int cols = J.cols();
            int nnz = J.nonZeros();

            // 在 GPU 上分配内存
            float* d_x, * d_y;
            int* d_csrRowPtr, * d_csrColInd;
            float* d_csrVal;

            cudaMalloc(&d_x, cols * sizeof(float));
            cudaMalloc(&d_y, rows * sizeof(float));
            cudaMalloc(&d_csrRowPtr, (rows + 1) * sizeof(int));
            cudaMalloc(&d_csrColInd, nnz * sizeof(int));
            cudaMalloc(&d_csrVal, nnz * sizeof(float));

            cudaMemcpy(d_x, x.data(), cols * sizeof(float), cudaMemcpyHostToDevice);
            cudaMemcpy(d_csrRowPtr, J.outerIndexPtr(), (rows + 1) * sizeof(int), cudaMemcpyHostToDevice);
            cudaMemcpy(d_csrColInd, J.innerIndexPtr(), nnz * sizeof(int), cudaMemcpyHostToDevice);
            cudaMemcpy(d_csrVal, J.valuePtr(), nnz * sizeof(float), cudaMemcpyHostToDevice);

            cusparseHandle_t handle;
            cusparseCreate(&handle);

            cusparseSpMatDescr_t matA;
            cusparseDnVecDescr_t vecX, vecY;
            cusparseCreateCsr(&matA, rows, cols, nnz, d_csrRowPtr, d_csrColInd, d_csrVal,
                              CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                              CUSPARSE_INDEX_BASE_ZERO, CUDA_R_32F);
            cusparseCreateDnVec(&vecX, cols, d_x, CUDA_R_32F);
            cusparseCreateDnVec(&vecY, rows, d_y, CUDA_R_32F);

            float alpha = 1.0f;
            float beta = 0.0f;

            size_t bufferSize = 0;
            void* d_buffer = nullptr;

            cusparseSpMV_bufferSize(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha, matA, vecX, &beta, vecY,
                                    CUDA_R_32F, CUSPARSE_SPMV_ALG_DEFAULT, &bufferSize);

            if (bufferSize > 0) {
                cudaMalloc(&d_buffer, bufferSize);
            }

            cusparseSpMV(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha, matA, vecX, &beta, vecY,
                         CUDA_R_32F, CUSPARSE_SPMV_ALG_DEFAULT, d_buffer);

            VectorXf y(rows);
            cudaMemcpy(y.data(), d_y, rows * sizeof(float), cudaMemcpyDeviceToHost);

            cudaFree(d_x);
            cudaFree(d_y);
            cudaFree(d_csrRowPtr);
            cudaFree(d_csrColInd);
            cudaFree(d_csrVal);
            cudaFree(d_buffer);
            cusparseDestroyDnVec(vecX);
            cusparseDestroyDnVec(vecY);
            cusparseDestroySpMat(matA);
            cusparseDestroy(handle);

            return std::move(y);
        }
#endif
        static inline void printSparseMatrix(const SparseMatrixCSR& matrix) {
            for (int k = 0; k < matrix.outerSize(); ++k) {
                for (SparseMatrixCSR::InnerIterator it(matrix, k); it; ++it) {
                    std::cout << "Element at (" << it.row() << ", " << it.col() << ") = " << it.value() << std::endl;
                }
            }
        }
        static inline void printCholesky(const Cholesky& matrix) {
            printSparseMatrix(choleskyToSparseMatrix(matrix));
        }
        static inline SparseMatrixCSR choleskyToSparseMatrix(const Cholesky& cholesky) {
            SparseMatrixCSR L = cholesky.matrixL();
            SparseMatrixCSR fullMatrix = L * L.transpose();
            return std::move(fullMatrix);
        }
        static inline void printVectorXf(const VectorXf& vec) {
            std::cout << vec << std::endl;
        }
    private:
        dtkPhysMassSpringSolver();
        dtkPhysMassSpringSolver(const dtkPhysMassSpring::Ptr& massSpring);

        void step();
        dtkPhysMassSpring::Ptr _system;
#ifdef DTK_CUDA
        CuSparseCholeskySolver<float>::Ptr _system_matrix;
#else
        Cholesky _system_matrix;
#endif

        // M, L, J matrices
        SparseMatrixCSR _M;/**< 质量稀疏矩阵  */
        SparseMatrixCSR _L;/**< Laplace稀疏矩阵，表示弹簧质点系统中质点与质点之间的连接关系，以及连接弹簧的刚度 */
        SparseMatrixCSR _J;/**< Jacobian稀疏矩阵，表示弹簧质点系统中每个质点与弹簧的连接关系，以及连接弹簧的刚度 */

        VectorXf _initial_state;    // q(0)
        VectorXf _current_state;  // q(n)
        VectorXf _prev_state;  // q(n-1)
        VectorXf _spring_directions; // d, spring directions
        VectorXf _inertial_term; /**< 惯性项 = M * y, y = (a + 1) * q(n) - a * q(n - 1), a = damp_factor */
        VectorXf _fext_force; /**< 外力项 */
        double _h2; /**< 时间步长的平方 */

        std::vector<float> _rest_lengths;  // 存储每个弹簧的初始长度
        std::vector<int> _spring_indices;  // 存储每个弹簧的两个顶点的索引（大小为 2 * num_springs）

#ifdef DTK_CUDA
        float* d_current_state;
        float* d_spring_directions;
        int* d_spring_indices;
        float* d_rest_lengths;
        float* d_inertial_term;
        float* d_fext_force;
        float* d_J_spring_directions;
        float* d_b;
#endif
        float _time_step;
    public:
#ifdef DTK_CUDA
        ~dtkPhysMassSpringSolver() {
            cudaFree(d_current_state);
            cudaFree(d_spring_directions);
            cudaFree(d_spring_indices);
            cudaFree(d_rest_lengths);
            cudaFree(d_inertial_term);
            cudaFree(d_fext_force);
            cudaFree(d_b);
            cudaFree(d_J_spring_directions);
        }
#endif
    };
}
#endif // SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGSOLVER_H