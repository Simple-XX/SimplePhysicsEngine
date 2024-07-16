/**
 * @file ClothSimulation.cpp
 * @brief ClothSimulation 实现
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
#include "ClothSimulation.h"

ClothSimulation::ClothSimulation(const dtk::dtkDouble2& gravity) {
    _gravity = gravity;
};

void ClothSimulation::move(const dtk::dtkDouble2& v) {

};

void ClothSimulation::clear() {

};

void ClothSimulation::step(double dt) {

};

bool ClothSimulation::is_pause() const { return _pause; };
void ClothSimulation::set_pause(bool pause) { _pause = pause; };