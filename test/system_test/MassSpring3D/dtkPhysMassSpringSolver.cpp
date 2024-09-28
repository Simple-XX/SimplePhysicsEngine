#include "dtkPhysMassSpringSolver.h"

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
    _h2 = _system->GetTimeStep() * _system->GetTimeStep();
    SparseMatrixCSR A = _M + _h2 * _L;

    // external force (gravity)
    const dtk::dtkDouble3& fext = _system->GetDefaultGravityAccel();
    Vector3f gravity(fext.x, fext.y, fext.z);
    _fext_force = gravity.replicate(_system->GetNumberOfMassPoints(), 1);

#ifdef DTK_CUDA
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(A.cols());
    const int nnz = static_cast<int>(A.nonZeros());

    std::cout << "=== Matrix : " << std::endl;
    std::cout << "Size       : " << n << " x " << m << std::endl;
    std::cout << "Non-zeros  : " << nnz << std::endl;

    SparseMatrixCSR Acsr = A; // solver supports CSR format
    // printSparseMatrix(Acsr);
    _system_matrix = CuSparseCholeskySolver<float>::create(n);

    bool doOrdering = true;
    if (doOrdering)
    {
        // compute permutation
        PermutationMatrix P;
        Ordering ordering;
        ordering(Acsr.selfadjointView<Eigen::Upper>(), P);

        // set permutation to solver
        _system_matrix->setPermutaion(n, P.indices().data());
    }

    _system_matrix->analyze(nnz, Acsr.outerIndexPtr(), Acsr.innerIndexPtr());

    _system_matrix->factorize(Acsr.valuePtr());

    if (_system_matrix->info() != CuSparseCholeskySolver<float>::SUCCESS)
    {
        std::cerr << "Factorize failed." << std::endl;
        std::exit(EXIT_FAILURE);
    }
#else
    _system_matrix.compute(A);  // Cholesky 分解
#endif

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

#ifdef DTK_CUDA
    // Initialize CUDA and allocate memory on GPU
    cudaMalloc(&d_current_state, _current_state.size() * sizeof(float));
    cudaMalloc(&d_spring_directions, _spring_directions.size() * sizeof(float));
    cudaMalloc(&d_rest_lengths, _rest_lengths.size() * sizeof(float));
    cudaMalloc(&d_spring_indices, _spring_indices.size() * sizeof(int));
    cudaMalloc(&d_fext_force, _fext_force.size() * sizeof(float));
    cudaMalloc(&d_b, _fext_force.size() * sizeof(float));
    cudaMalloc(&d_J_spring_directions, _fext_force.size() * sizeof(float));

    // Copy initial data from CPU to GPU
    cudaMemcpy(d_current_state, _current_state.data(), _current_state.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_rest_lengths, _rest_lengths.data(), _rest_lengths.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_spring_indices, _spring_indices.data(), _spring_indices.size() * sizeof(int), cudaMemcpyHostToDevice);
#endif

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
#ifdef DTK_CUDA
    VectorXf v_term = (damping_factor + 1) * (_current_state)-damping_factor * _prev_state;
    _inertial_term = cusparse_multiply(_M, v_term);
#else
    _inertial_term = _M * ((damping_factor + 1) * (_current_state)-damping_factor * _prev_state);
#endif

    _prev_state = _current_state;

#ifdef DTK_CUDA
    cudaMemcpy(d_inertial_term, _inertial_term.data(), _inertial_term.size() * sizeof(float), cudaMemcpyHostToDevice);
#endif

    // perform steps
    for (unsigned int i = 0; i < iter_num; i++)
        step();
}

void dtk::dtkPhysMassSpringSolver::step() {
    // local step
#ifdef DTK_CUDA
    int num_springs = _system->GetNumberOfSprings();
    run_local_step_with_cuda(d_current_state, d_spring_directions, d_spring_indices, d_rest_lengths, num_springs);
    cudaMemcpy(_spring_directions.data(), d_spring_directions, _spring_directions.size() * sizeof(float), cudaMemcpyDeviceToHost);
#else
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
#endif
    // global step
#ifdef DTK_CUDA
    // step(1)

    // auto start = std::chrono::high_resolution_clock::now();

    VectorXf _J_spring_directions = cusparse_multiply(_J, _spring_directions);
    VectorXf b = _inertial_term + _h2 * (_J_spring_directions + _fext_force);

    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = end - start;
    // std::cout << "step(1) time: " << elapsed.count() << " seconds" << std::endl;

    // step(2)

    // start = std::chrono::high_resolution_clock::now();

    VectorR xhatGPU(b.size());
    _system_matrix->solve(b.data(), xhatGPU.data());
    _current_state = Eigen::Map<VectorXf>(xhatGPU.data(), xhatGPU.size());
    cudaMemcpy(d_current_state, _current_state.data(), _current_state.size() * sizeof(float), cudaMemcpyHostToDevice);

    // end = std::chrono::high_resolution_clock::now();
    // elapsed = end - start;
    // std::cout << "step(2) time: " << elapsed.count() << " seconds" << std::endl;
#else
    VectorXf b = _inertial_term + _h2 * (_J * _spring_directions + _fext_force);
    _current_state = _system_matrix.solve(b);
#endif
}


void dtk::dtkPhysMassSpringSolver::satisfy(ClothDropType type) {
    auto satisfySphere = [&](const float& radius, const Eigen::Vector3f& center) {
#ifdef DTK_CUDA
        int num_points = _system->GetNumberOfMassPoints();
        size_t size = num_points * 3 * sizeof(float);
        run_satisfy_sphere_with_cuda(d_current_state, num_points, radius, make_float3(center[0], center[1], center[2]));
        cudaMemcpy(_current_state.data(), d_current_state, size, cudaMemcpyDeviceToHost);
#else
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
#endif
        };


    if (type == Sphere) {
        satisfySphere(0.64f, Eigen::Vector3f(-0.5, 0, -1));
        satisfySphere(0.64f, Eigen::Vector3f(0, 0.5, -2));
    }
}