
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

#ifndef SIMPLEPHYSICSENGINE_CONSTANTS_H
#define SIMPLEPHYSICSENGINE_CONSTANTS_H

namespace Constants {

static constexpr const float WIDTH = 3;
static constexpr const float HEIGHT = 1.5;

static constexpr const float SCALE = 400;

static constexpr const int RENDER_WIDTH = SCALE * WIDTH;
static constexpr const int RENDER_HEIGHT = SCALE * HEIGHT;

static constexpr const float WINDOW_SCALE = 1.0f;

static constexpr const float TIMESTEP = 0.0001f;

static constexpr const int NUMBER_PARTICLES = 70;

static constexpr const float REST_DENSITY = 1000;

static constexpr const float STIFFNESS = 10000;
static constexpr const float VISCOCITY = 12000;
static constexpr const float TENSION = 10000.0f;

static constexpr const float GRAVITY = -12000;

static constexpr const float PARTICLE_SPACING = 1.0f / NUMBER_PARTICLES;
static constexpr const float PARTICLE_VOLUME =
    PARTICLE_SPACING * PARTICLE_SPACING;
static constexpr const float PARTICLE_MASS = PARTICLE_VOLUME * REST_DENSITY;
static constexpr const float KERNEL_RANGE = 2 * PARTICLE_SPACING;

} // namespace Constants

#endif /* SIMPLEPHYSICSENGINE_CONSTANTS_H */
