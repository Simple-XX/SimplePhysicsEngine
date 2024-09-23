#include "dtkPhysMassSpringSolver.h"
#include "step.cuh"

dtk::dtkPhysMassSpringSolver::dtkPhysMassSpringSolver() {
}

dtk::dtkPhysMassSpringSolver::dtkPhysMassSpringSolver(const dtk::dtkPhysMassSpring::Ptr& massSpringSystem) {
    _system = massSpringSystem;
    _time_step = _system->GetTimeStep();

    // current state
    _current_state.resize(3 * _system->GetNumberOfMassPoints());
    for (dtk::dtkID i = 0; i < _system->GetNumberOfMassPoints(); i++) {
        dtk::dtkPhysMassPoint* massPoint = _system->GetMassPoint(i);
        _current_state[3 * i] = massPoint->GetPosition()[0];
        _current_state[3 * i + 1] = massPoint->GetPosition()[1];
        _current_state[3 * i + 2] = massPoint->GetPosition()[2];
    }
    _prev_state = _current_state;
    _initial_state = _current_state;
    _spring_directions.resize(3 * _system->GetNumberOfSprings());

    // M 
    TripletList MTriplets;
    _M.resize(3 * _system->GetNumberOfMassPoints(), 3 * _system->GetNumberOfMassPoints());
    for (dtk::dtkID i = 0; i < _system->GetNumberOfMassPoints(); i++) {
        for (dtk::dtkID j = 0; j < 3; j++) {
            MTriplets.push_back(Triplet(3 * i + j, 3 * i + j, _system->GetMassPoint(i)->GetMass()));
        }
    }
    _M.setFromTriplets(MTriplets.begin(), MTriplets.end());

    // L
    TripletList LTriplets;
    _L.resize(3 * _system->GetNumberOfMassPoints(), 3 * _system->GetNumberOfMassPoints());
    for (dtk::dtkID i = 0;i < _system->GetNumberOfSprings();i++) {
        double stiffness = _system->GetSpring(i)->GetStiffness();
        dtk::dtkPhysSpring* spring = _system->GetSpring(i);
        dtk::dtkID id1 = spring->GetFirstVertex()->GetPointID();// TODO GetFirstVertex修改成GetFirstPoint();
        dtk::dtkID id2 = spring->GetSecondVertex()->GetPointID();
        for (dtk::dtkID j = 0; j < 3; j++) {
            LTriplets.push_back(Triplet(3 * id1 + j, 3 * id1 + j, 1 * stiffness));
            LTriplets.push_back(Triplet(3 * id1 + j, 3 * id2 + j, -1 * stiffness));
            LTriplets.push_back(Triplet(3 * id2 + j, 3 * id1 + j, -1 * stiffness));
            LTriplets.push_back(Triplet(3 * id2 + j, 3 * id2 + j, 1 * stiffness));
        }
    }
    _L.setFromTriplets(LTriplets.begin(), LTriplets.end());

    // J
    TripletList JTriplets;
    _J.resize(3 * _system->GetNumberOfMassPoints(), 3 * _system->GetNumberOfSprings());
    for (dtk::dtkID i = 0;i < _system->GetNumberOfSprings();i++) {
        double stiffness = _system->GetSpring(i)->GetStiffness();
        dtk::dtkPhysSpring* spring = _system->GetSpring(i);
        dtk::dtkID id1 = spring->GetFirstVertex()->GetPointID();
        dtk::dtkID id2 = spring->GetSecondVertex()->GetPointID();
        for (unsigned int j = 0; j < 3; j++) {
            JTriplets.push_back(
                Triplet(3 * id1 + j, 3 * i + j, 1 * stiffness));
            JTriplets.push_back(
                Triplet(3 * id2 + j, 3 * i + j, -1 * stiffness));
        }
    }
    _J.setFromTriplets(JTriplets.begin(), JTriplets.end());

    // pre-factor 
    double h2 = _system->GetTimeStep() * _system->GetTimeStep();
    SparseMatrix A = _M + h2 * _L;
    _system_matrix.compute(A);


    int num_springs = _system->GetNumberOfSprings();
    _rest_lengths.resize(num_springs);
    _spring_indices.resize(2 * num_springs);

    // 初始化弹簧数据
    for (int i = 0; i < num_springs; ++i) {
        dtk::dtkPhysSpring* spring = _system->GetSpring(i);
        _rest_lengths[i] = spring->GetRestLength();
        _spring_indices[2 * i] = spring->GetFirstVertex()->GetPointID();
        _spring_indices[2 * i + 1] = spring->GetSecondVertex()->GetPointID();
    }

    // std::cout << "M: " << std::endl;
    // printSparseMatrix(_M);
    // std::cout << "L: " << std::endl;
    // printSparseMatrix(_L);
    // std::cout << "J: " << std::endl;
    // printSparseMatrix(_J);
    // std::cout << "A: " << std::endl;
    // printSparseMatrix(A);
}

void dtk::dtkPhysMassSpringSolver::solve(unsigned int iter_num) {
    float damping_factor = _system->GetDefaultPointDamp();

    // update inertial term
    _inertial_term = _M * ((damping_factor + 1) * (_current_state)-damping_factor * _prev_state);
    _prev_state = _current_state;

    // perform steps
    bool use_cuda = true;
    for (unsigned int i = 0; i < iter_num; i++) {
        step(use_cuda);
    }
}

extern "C" void run_local_step_kernel(const float* current_state, float* spring_directions, const float* rest_lengths, const int* spring_indices, int num_springs);

void dtk::dtkPhysMassSpringSolver::step(bool use_cuda) {
    if (use_cuda) {
        // ***** CUDA Setup ***** //
        cusparseHandle_t handle;
        cusparseCreate(&handle);

        int num_springs = _system->GetNumberOfSprings();
        int num_points = _system->GetNumberOfMassPoints();
        float h2 = _system->GetTimeStep() * _system->GetTimeStep();

        // 分配 GPU 内存
        float* d_current_state, * d_spring_directions, * d_inertial_term, * d_fext_force, * d_b;
        float* d_rest_lengths;
        int* d_spring_indices;

        cudaMalloc(&d_current_state, _current_state.size() * sizeof(float));
        cudaMalloc(&d_spring_directions, _spring_directions.size() * sizeof(float));
        cudaMalloc(&d_inertial_term, _inertial_term.size() * sizeof(float));
        cudaMalloc(&d_fext_force, num_points * 3 * sizeof(float)); // Assuming 3D points
        cudaMalloc(&d_b, _inertial_term.size() * sizeof(float));
        cudaMalloc(&d_rest_lengths, num_springs * sizeof(float));
        cudaMalloc(&d_spring_indices, 2 * num_springs * sizeof(int)); // 每个弹簧有两个顶点

        // 将数据从 CPU 拷贝到 GPU
        cudaMemcpy(d_current_state, _current_state.data(), _current_state.size() * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_rest_lengths, _rest_lengths.data(), num_springs * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_spring_indices, _spring_indices.data(), 2 * num_springs * sizeof(int), cudaMemcpyHostToDevice);

        // 调用 CUDA 函数执行 local step
        run_local_step_kernel(d_current_state, d_spring_directions, d_rest_lengths, d_spring_indices, num_springs);

        // 拷贝结果回 CPU
        cudaMemcpy(_spring_directions.data(), d_spring_directions, _spring_directions.size() * sizeof(float), cudaMemcpyDeviceToHost);

        // ***** 外力计算 ***** //
        dtk::dtkDouble3 fext = _system->GetDefaultGravityAccel();
        Vector3f fext_vector(fext.x, fext.y, fext.z);
        VectorXf fext_force = fext_vector.replicate(num_points, 1);
        cudaMemcpy(d_fext_force, fext_force.data(), num_points * 3 * sizeof(float), cudaMemcpyHostToDevice);

        // ***** Right Hand Side (RHS) 的计算 ***** //
        VectorXf rhs = _inertial_term + h2 * _J * _spring_directions + h2 * fext_force;
        cudaMemcpy(d_b, rhs.data(), rhs.size() * sizeof(float), cudaMemcpyHostToDevice);

        // ***** cuSPARSE 矩阵求解 ***** //
        Eigen::SparseMatrix<float> system_matrix_eigen = _system_matrix.matrixL();  // 获取分解的下三角矩阵
        int nnz = system_matrix_eigen.nonZeros();
        int rows = system_matrix_eigen.rows();
        int cols = system_matrix_eigen.cols();

        // 生成 CSR 格式的数据
        std::vector<int> csrRowPtr(rows + 1), csrColInd(nnz);
        std::vector<float> csrVal(nnz);
        int idx = 0;
        for (int k = 0; k < system_matrix_eigen.outerSize(); ++k) {
            csrRowPtr[k] = idx;
            for (Eigen::SparseMatrix<float>::InnerIterator it(system_matrix_eigen, k); it; ++it) {
                csrVal[idx] = it.value();
                csrColInd[idx] = it.col();
                idx++;
            }
        }
        csrRowPtr[rows] = nnz;

        // 在 GPU 上分配 CSR 矩阵
        int* d_csrRowPtr, * d_csrColInd;
        float* d_csrVal;
        cudaMalloc(&d_csrRowPtr, (rows + 1) * sizeof(int));
        cudaMalloc(&d_csrColInd, nnz * sizeof(int));
        cudaMalloc(&d_csrVal, nnz * sizeof(float));
        cudaMemcpy(d_csrRowPtr, csrRowPtr.data(), (rows + 1) * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_csrColInd, csrColInd.data(), nnz * sizeof(int), cudaMemcpyHostToDevice);
        cudaMemcpy(d_csrVal, csrVal.data(), nnz * sizeof(float), cudaMemcpyHostToDevice);

        // 使用 cuSPARSE 进行求解
        float* d_x;
        cudaMalloc(&d_x, rhs.size() * sizeof(float));

        cusparseMatDescr_t descr;
        cusparseCreateMatDescr(&descr);

        // 定义 alpha 和 beta
        float alpha = 1.0f;
        float beta = 0.0f;

        // 使用 cuSPARSE API 进行矩阵-向量乘法
        cusparseSpMatDescr_t matA;
        cusparseDnVecDescr_t vecX, vecY;
        cusparseCreateCsr(&matA, rows, cols, nnz, d_csrRowPtr, d_csrColInd, d_csrVal,
                          CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                          CUSPARSE_INDEX_BASE_ZERO, CUDA_R_32F);
        cusparseCreateDnVec(&vecX, rhs.size(), d_b, CUDA_R_32F);
        cusparseCreateDnVec(&vecY, rhs.size(), d_x, CUDA_R_32F);

        cusparseSpMV(handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &alpha, matA, vecX, &beta, vecY,
                     CUDA_R_32F, CUSPARSE_SPMV_ALG_DEFAULT, nullptr);

        // 将解拷贝回 CPU
        cudaMemcpy(_current_state.data(), d_x, rhs.size() * sizeof(float), cudaMemcpyDeviceToHost);

        // 释放 GPU 资源
        cudaFree(d_current_state);
        cudaFree(d_spring_directions);
        cudaFree(d_inertial_term);
        cudaFree(d_fext_force);
        cudaFree(d_b);
        cudaFree(d_x);
        cudaFree(d_csrRowPtr);
        cudaFree(d_csrColInd);
        cudaFree(d_csrVal);
        cudaFree(d_rest_lengths);
        cudaFree(d_spring_indices);
        cusparseDestroyMatDescr(descr);
        cusparseDestroy(handle);
    }
    else {
        // 原有 CPU 代码
        for (dtk::dtkID id = 0; id < _system->GetNumberOfSprings(); id++) {
            dtk::dtkPhysSpring* spring = _system->GetSpring(id);
            dtk::dtkID id1 = spring->GetFirstVertex()->GetPointID();
            dtk::dtkID id2 = spring->GetSecondVertex()->GetPointID();
            double rest_length = spring->GetRestLength();
            Vector3f p12(
                _current_state[3 * id1 + 0] - _current_state[3 * id2 + 0],
                _current_state[3 * id1 + 1] - _current_state[3 * id2 + 1],
                _current_state[3 * id1 + 2] - _current_state[3 * id2 + 2]
            );

            p12.normalize();
            _spring_directions[3 * id + 0] = rest_length * p12[0];
            _spring_directions[3 * id + 1] = rest_length * p12[1];
            _spring_directions[3 * id + 2] = rest_length * p12[2];
        }

        float h2 = _system->GetTimeStep() * _system->GetTimeStep();
        dtk::dtkDouble3 fext = _system->GetDefaultGravityAccel();
        VectorXf fext_force = VectorXf(Vector3f(fext.x, fext.y, fext.z).replicate(_system->GetNumberOfMassPoints(), 1));

        VectorXf b = _inertial_term + h2 * _J * _spring_directions + h2 * fext_force;
        _current_state = _system_matrix.solve(b);
    }
}


void dtk::dtkPhysMassSpringSolver::satisfy(ClothDropType type) {
    if (type == Sphere) {
        const float radius = 0.64f;
        const Eigen::Vector3f center(0, 0, -1);

        for (int i = 0; i < _system->GetNumberOfMassPoints(); i++) {
            Vector3f p(
                _current_state[3 * i + 0] - center[0],
                _current_state[3 * i + 1] - center[1],
                _current_state[3 * i + 2] - center[2]
            );

            if (p.norm() < radius) {
                p.normalize();
                p = radius * p;
            }
            else continue;

            for (int j = 0; j < 3; j++) {
                _current_state[3 * i + j] = p[j] + center[j];
            }
        }
    }
    else {
        int fix_idx = 0;
        for (int i = 0; i < 3; i++)
            _current_state[fix_idx + i] = _initial_state[fix_idx + i];
    }
}