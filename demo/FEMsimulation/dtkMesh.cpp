#include "dtkMesh.h"

dtkMesh::dtkMesh()
{

}

dtkMesh::dtkMesh(int x_node, int y_node) :
	n_node_x_(x_node), n_node_y_(y_node), n_node_(x_node* y_node), n_fem_element_((n_node_x_ - 1)* (n_node_y_ - 1) * 2),
	points_(n_node_), pre_points_(n_node_),
	points_v_(n_node_), points_force_(n_node_),
	mesh_table_(n_fem_element_, std::vector<int>(3, 0)), total_energy_(0),
	pre_total_energy_(0)
{
	B_.resize(n_fem_element_);
	dim_ = 2;
	element_v_ = 0.02f;

	Young_E_ = 31000.0f;
	Poisson_r_ = 0.3f;
	Lame_parameter_1_ = Young_E_ / (2 * (1 + Poisson_r_));
	Lame_parameter_2_ = Young_E_ * Poisson_r_ / ((1 + Poisson_r_) * (1 - 2 * Poisson_r_));
	node_mass_ = 1.0f;
	deltax = (1.0 / 32);
}

Matrix2f dtkMesh::compute_D(int i) {
	int a = mesh_table_[i][0];
	int b = mesh_table_[i][1];
	int c = mesh_table_[i][2];

	Matrix2f ans;
	ans(0, 0) = points_[a][0] - points_[c][0];
	ans(0, 1) = points_[b][0] - points_[c][0];
	ans(1, 0) = points_[a][1] - points_[c][1];
	ans(1, 1) = points_[b][1] - points_[c][1];
	return ans;
}

void dtkMesh::compute_B() {
	for (int i = 0; i < n_fem_element_; ++i) {
		this->B_[i] = compute_D(i).inverse();
	}
}

Matrix2f dtkMesh::compute_P(int i) {
	Matrix2f D = compute_D(i);
	Matrix2f F = D * B_[i];

	Matrix2f F_T = F.transpose().inverse();

	float J = fmax(0.5f, F.determinant()); /**< 形变率 */

	return Lame_parameter_1_ * (F - F_T) + Lame_parameter_2_ * log(J) * F_T;
}

void dtkMesh::compute_force() {

	for (int i = 0; i < n_node_; ++i) {
		this->points_force_[i] = Vector2f(0.0f, -10.0f * node_mass_);
	}

	for (int i = 0; i < n_fem_element_; ++i) {

		Matrix2f P = compute_P(i);
		Matrix2f H = -element_v_ * (P * (this->B_[i].transpose()));

		Vector2f h1 = Vector2f(H(0, 0), H(1, 0));
		Vector2f h2 = Vector2f(H(0, 1), H(1, 1));

		int a = this->mesh_table_[i][0];
		int b = this->mesh_table_[i][1];
		int c = this->mesh_table_[i][2];

		this->points_force_[a] += h1;
		this->points_force_[b] += h2;
		this->points_force_[c] += -(h1 + h2);
	}
}

void dtkMesh::compute_total_energy() {
	this->total_energy_ = 0.0f;
	for (int i = 0; i < n_fem_element_; ++i) {
		Matrix2f D = compute_D(i);
		Matrix2f F = D * B_[i];

		//NeoHooken
		float I1 = (F * F.transpose()).trace();
		float J = fmax(0.2f, (float)F.determinant()); /**< 形变率 */


		float element_energy_density = 0.5 * Lame_parameter_1_ * (I1 - dim_) - Lame_parameter_1_ * log(J) + 0.5 * Lame_parameter_2_ * log(J) * log(J);
		this->total_energy_ += element_energy_density * element_v_;
	}
}

void dtkMesh::Init()
{
	for (int i = 0; i < n_node_x_; ++i) {
		for (int j = 0; j < n_node_y_; ++j) {
			int idx = mesh(i, j);

			this->points_[idx][0] = 0.1f + i * deltax * 0.5f;
			this->points_[idx][1] = 0.5f + j * deltax * 0.5f + i * deltax * 0.1f;
			this->points_v_[idx][0] = 0.0f;
			this->points_v_[idx][1] = -1.0f;
		}
	}

	//this->pre_points = points;

	for (int i = 0; i < n_node_x_ - 1; ++i) {
		for (int j = 0; j < n_node_y_ - 1; ++j) {
			//element id
			int eidx = (i * (n_node_y_ - 1) + j) * 2;
			this->mesh_table_[eidx][0] = mesh(i, j);
			this->mesh_table_[eidx][1] = mesh(i + 1, j);
			this->mesh_table_[eidx][2] = mesh(i, j + 1);

			eidx = (i * (n_node_y_ - 1) + j) * 2 + 1;
			this->mesh_table_[eidx][0] = mesh(i, j + 1);
			this->mesh_table_[eidx][1] = mesh(i + 1, j + 1);
			this->mesh_table_[eidx][2] = mesh(i + 1, j);
		}
	}

	compute_B();
}
