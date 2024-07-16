/**
 * @file ClothSimulation.h
 * @brief ClothSimulation 头文件
 * @author Chance
 * @version 1.0
 * @date 2024-07-12
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2024-07-12<td>Chance<td>创建文件
 * </table>
 */
#ifndef SIMPLEPHYSICSENGINE_CLOTHSIMULATION_H
#define SIMPLEPHYSICSENGINE_CLOTHSIMULATION_H

#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <dtkJoint.h>
#include <dtkScene.h>

class ClothSimulation {
public:
    ClothSimulation(const dtk::dtkDouble2& gravity);
    ~ClothSimulation() = default;

    void move(const dtk::dtkDouble2& v);

    void clear();
    void step(double dt);

    bool is_pause() const;
    void set_pause(bool pause);

private:
    // 是否暂停
    bool _pause{ false };
    // 重力
    dtk::dtkDouble2 _gravity;
    size_t _iterations{ 10 };
};

class dtkFactory {
public:

};

#endif /* SIMPLEPHYSICSENGINE_CLOTHSIMULATION_H */
