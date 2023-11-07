
/**
 * @file Mesh.h
 * @brief Mesh 头文件
 * @author Zone.N (Zone.Niuzh@hotmail.com)
 * @version 1.0
 * @date 2023-10-31
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2023-10-31<td>Zone.N<td>迁移到 doxygen
 * </table>
 */

#ifndef SIMPLEPHYSICSENGINE_MESH_H
#define SIMPLEPHYSICSENGINE_MESH_H

#include <vector>

#include <Eigen/Dense>
#include <dtkRigidBody.h>

#include "Object.h"

class Mesh : public Object {
public:
  using ptr = std::shared_ptr<Mesh>;
  // 记录所有点上一个时间点的位置(n_node)
  std::vector<Eigen::Vector2f> pre_points_;
  Eigen::Vector2f position;

  // 微元本身长度的逆(n_node)
  std::vector<Eigen::Matrix2f> B_;
  // 总势能
  float total_energy_;
  // 之前的总势能
  float pre_total_energy_;

  // 物体的节点数
  int n_node_x_, n_node_y_, n_node_;
  // 有限元的个数
  int n_fem_element_;

  int dim_;
  // 微元体积
  float element_v_;
  // 杨氏模量
  float Young_E_;
  // 泊松比 [0 - 0.5]
  float Poisson_r_;
  float Lame_parameter_1_;
  float Lame_parameter_2_;
  int mID;

  Eigen::Matrix2f compute_D(int i);
  Eigen::Matrix2f compute_P(int i);
  void compute_B();
  void updateShell();

  inline int vertex(int i) {
    int ret = 0;
    if (i >= 0 && i < n_node_x_ - 1)
      ret = mesh(i, 0);
    else if (i >= n_node_x_ - 1 && i < n_node_x_ + n_node_y_ - 2)
      ret = mesh(n_node_x_ - 1, i - n_node_x_ + 1);
    else if (i >= n_node_x_ + n_node_y_ - 2 &&
             i < 2 * n_node_x_ + n_node_y_ - 3)
      ret = mesh(2 * n_node_x_ + n_node_y_ - 3 - i, n_node_y_ - 1);
    else if (i >= 2 * n_node_x_ + n_node_y_ - 3 &&
             i < 2 * n_node_x_ + 2 * n_node_y_ - 4)
      ret = mesh(0, 2 * n_node_x_ + 2 * n_node_y_ - 4 - i);

    return ret;
  }

  inline int mesh(int i, int j) { return i * n_node_y_ + j; }

  // 记录所有点的位置(n_node)
  std::vector<Eigen::Vector2f> points_;
  // 记录所有点上的速度(n_node)
  std::vector<Eigen::Vector2f> points_v_;
  // 记录所有点上的力(n_node)
  std::vector<Eigen::Vector2f> points_force_;
  // 三角微元下标(n_fem_element, std::vector<int>(3,0))
  std::vector<std::vector<int>> mesh_table_;

  float node_mass_;
  float deltax;

  dtk::dtkPolygonRigidBody::ptr shell;

  Mesh();
  Mesh(int id, int x_node, int y_node, Eigen::Vector2f pos);
  void Init();
  float getEnergy() { return total_energy_; }

  void compute_force();
  void compute_total_energy();
};

#endif /* SIMPLEPHYSICSENGINE_MESH_H */
