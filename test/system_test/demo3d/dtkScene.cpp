
/**
 * @file dtkScene.cpp
 * @brief dtkScene 实现
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

#include "dtkScene.h"

dtkScene::dtkScene(unsigned int width, unsigned int height)
    : State(SCENE_ACTIVE), Keys(), KeysProcessed(), Width(width),
      Height(height) {}

dtkScene::~dtkScene() {}

void dtkScene::Init() {
  // TODO: load shaders

  // TODO: configure shaders

  // TODO: load textures

  // TODO: set render-specific controls

  // TODO: configure Scene objects

  // TODO: audio
}

void dtkScene::Update(float dt) {
  // TODO: update objects
  // TODO: check for object collisions
}

void dtkScene::ProcessInput(float dt) {
  // TODO: process input(keys)
  /*

  if (this->State == SCENE_ACTIVE)
  {
      if (this->Keys[GLFW_KEY_SPACE] && !this->KeysProcessed[GLFW_KEY_SPACE])
      {
          this->State = SCENE_PAUSE;
          this->KeysProcessed[GLFW_KEY_SPACE] = true;
      }
  }
  if (this->State == SCENE_PAUSE)
  {
      if (this->Keys[GLFW_KEY_ENTER])
      {
          this->State = SCENE_ACTIVE;
          this->KeysProcessed[GLFW_KEY_ENTER] = true;
      }
  }
  */
}

void dtkScene::Render() {}

// collision detection

void dtkScene::DoCollisions() {}