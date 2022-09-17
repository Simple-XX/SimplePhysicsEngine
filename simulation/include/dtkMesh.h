#ifndef DTKMESH_H
#define DTKMESH_H
#include <Eigen\Dense>
#include "dtkObject.h"
#include "dtkRigidBody.h"
#include <vector>
using namespace Eigen;
class dtkMesh : public dtkObject
{
public:
	using ptr = std::shared_ptr<dtkMesh>;
	std::vector<Vector2f> pre_points_; //记录所有点上一个时间点的位置(n_node)
	Vector2f position;

	std::vector<Eigen::Matrix2f> B_; //微元本身长度的逆(n_node)
	float total_energy_; //总势能
	float pre_total_energy_; //之前的总势能

	int n_node_x_, n_node_y_, n_node_; //物体的节点数
	int n_fem_element_; //有限元的个数

	int dim_;
	float element_v_; /**< 微元体积 */
	float Young_E_; /**< 杨氏模量 */
	float Poisson_r_; /**< 泊松比 [0 - 0.5] */
	float Lame_parameter_1_;
	float Lame_parameter_2_;
	int mID;

	Matrix2f compute_D(int i);
	Matrix2f compute_P(int i);
	void compute_B();
	void updateShell();

	inline int vertex(int i)
	{
		if (i >= 0 && i < n_node_x_ - 1)
			return mesh(i, 0);
		else if (i >= n_node_x_ - 1 && i < n_node_x_ + n_node_y_ - 2)
			return mesh(n_node_x_ - 1, i - n_node_x_ + 1);
		else if (i >= n_node_x_ + n_node_y_ - 2 && i < 2 * n_node_x_ + n_node_y_ - 3)
			return mesh(2 * n_node_x_ + n_node_y_ - 3 - i, n_node_y_ - 1);
		else if (i >= 2 * n_node_x_ + n_node_y_ - 3 && i < 2 * n_node_x_ + 2 * n_node_y_ - 4)
			return mesh(0, 2 * n_node_x_ + 2 * n_node_y_ - 4 - i);
	}

	inline int mesh(int i, int j) { return i * n_node_y_ + j; }

	std::vector<Eigen::Vector2f> points_; //记录所有点的位置(n_node)
	std::vector<Vector2f> points_v_; //记录所有点上的速度(n_node)
	std::vector<Eigen::Vector2f> points_force_; //记录所有点上的力(n_node)
	std::vector<std::vector<int>> mesh_table_; //三角微元下标(n_fem_element, std::vector<int>(3,0))

	float node_mass_;
	float deltax;

	dtk::dtkPolygonRigidBody::ptr shell;

	dtkMesh();
	dtkMesh(int id, int x_node, int y_node, Vector2f pos);
	void Init();
	float getEnergy() { return total_energy_; }

	void compute_force();
	void compute_total_energy();
};

#endif