
/**
 * @file dtkScene.h
 * @brief  dtkScene 头文件
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

/*
 * @Author: tom: https://github.com/TOMsworkspace
 * @Date: 2021-09-03 15:18:24
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 16:35:44
 */

#ifndef SIMPLEPHYSICSENGINE_DTKSCENE_H
#define SIMPLEPHYSICSENGINE_DTKSCENE_H

namespace dtk {

// Represents the current state of the scene
enum dtkSceneState { SCENE_ACTIVE, SCENE_PAUSE };

// Scene holds all Scene-related state and functionality.
// Combines all Scene-related data into a single class for
// easy access to each of the components and manageability.
class dtkScene {
public:
  // Scene state
  dtkSceneState State;
  bool Keys[1024];
  bool KeysProcessed[1024];
  unsigned int Width, Height;

  // constructor/destructor
  dtkScene(unsigned int width, unsigned int height);

  ~dtkScene();
  // initialize Scene state (load all shaders/textures)
  void Init();
  // Scene loop
  virtual void ProcessInput(float dt) = 0;
  virtual void Update(float dt) = 0;
  virtual void Render() = 0;
  virtual void DoCollisions() = 0;
};
}; // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKSCENE_H */
