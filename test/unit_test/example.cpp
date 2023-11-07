
/**
 * @file example.cpp
 * @brief 测试示例
 * @author Zone.N (Zone.Niuzh@hotmail.com)
 * @version 1.0
 * @date 2023-10-31
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2023-10-31<td>Zone.N<td>创建文件
 * </table>
 */

#include <chrono>
#include <cmath>
#include <iostream>

#include <GL/freeglut.h>
#include <dtkMatrix.h>
#include <gtest/gtest.h>

TEST(example, 示例) {
  EXPECT_TRUE(true == true);

  dtk::dtkMatrix22 matrix1;
  dtk::dtkMatrix22 matrix2;
  EXPECT_EQ(matrix1.length(), matrix2.length());

}

