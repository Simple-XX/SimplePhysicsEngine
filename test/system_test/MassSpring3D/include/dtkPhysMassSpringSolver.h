#ifndef SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGSOLVER_H
#define SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGSOLVER_H
#include "dtkPhysMassPoint.h"
#include "dtkPhysMassSpring.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <random>

namespace dtk {
    class dtkPhysMassSpringSolver {
    private:
        typedef Eigen::Vector3f Vector3f;
        typedef Eigen::VectorXf VectorXf;
        typedef Eigen::SparseMatrix<float> SparseMatrix;
        typedef Eigen::SimplicialLLT<Eigen::SparseMatrix<float> > Cholesky;
        typedef Eigen::Map<Eigen::VectorXf> Map;
        typedef std::pair<unsigned int, unsigned int> Edge;
        typedef Eigen::Triplet<float> Triplet;
        typedef std::vector<Triplet> TripletList;
    public:
        typedef std::shared_ptr<dtkPhysMassSpringSolver> Ptr;

        static dtkPhysMassSpringSolver::Ptr New() {
            return Ptr(new dtkPhysMassSpringSolver());
        }
        static dtkPhysMassSpringSolver::Ptr New(const dtkPhysMassSpring::Ptr& massSpring) {
            return Ptr(new dtkPhysMassSpringSolver(massSpring));
        }

        void solve(unsigned int iter_num);
        float* getVertexBuffer() { return _current_state.data(); };

        static void printSparseMatrix(const SparseMatrix& matrix) {
            for (int k = 0; k < matrix.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(matrix, k); it; ++it) {
                    std::cout << "Element at (" << it.row() << ", " << it.col() << ") = " << it.value() << std::endl;
                }
            }
        }
    private:
        dtkPhysMassSpringSolver();
        dtkPhysMassSpringSolver(const dtkPhysMassSpring::Ptr& massSpring);

        void localStep();
        void globalStep();

        Cholesky _system_matrix;
        dtkPhysMassSpring::Ptr _system;

        // M, L, J matrices
        SparseMatrix _M;/**< 质量稀疏矩阵  */
        SparseMatrix _L;/**< Laplace稀疏矩阵，表示弹簧质点系统中质点与质点之间的连接关系，以及连接弹簧的刚度 */
        SparseMatrix _J;/**< Jacobian稀疏矩阵，表示弹簧质点系统中每个质点与弹簧的连接关系，以及连接弹簧的刚度 */

        VectorXf _current_state;  // q(n)
        VectorXf _prev_state;  // q(n-1)
        VectorXf _spring_directions; // d, spring directions
        VectorXf _inertial_term; /**< 惯性项 = M * y, y = (a + 1) * q(n) - a * q(n - 1), a = damp_factor */

        float _time_step;
    };
}
#endif // SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGSOLVER_H