
/**
 * @file Constants.h
 * @brief Constants 头文件
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

#pragma once

namespace Constants {
extern const float WIDTH;
extern const float HEIGHT;

extern const float SCALE; // pixels per meter

extern const int RENDER_WIDTH;
extern const int RENDER_HEIGHT;

extern const float WINDOW_SCALE;

extern const float TIMESTEP;

extern const int NUMBER_PARTICLES; // Along one dimension

extern const float REST_DENSITY;

extern const float STIFFNESS;
extern const float VISCOCITY;
extern const float TENSION;

extern const float GRAVITY;

extern const float PARTICLE_SPACING;
extern const float PARTICLE_VOLUME;
extern const float PARTICLE_MASS;
extern const float KERNEL_RANGE;
} // namespace Constants