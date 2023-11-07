
/**
 * @file object.h
 * @brief object 头文件
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

#ifndef SIMPLEPHYSICSENGINE_OBJECT_H
#define SIMPLEPHYSICSENGINE_OBJECT_H

#include <Eigen/Dense>

class Object {
protected:
  Eigen::Vector2f center;

public:
  virtual void Init() = 0;
};

#endif /* SIMPLEPHYSICSENGINE_OBJECT_H */
