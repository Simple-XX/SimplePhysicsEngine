#include <GL/glew.h>
#include <GL/freeglut.h>

enum SceneState { SCENE_ACTIVE, SCENE_PAUSE };
enum SceneVisibility { SCENE_VISIBLE, SCENE_HIDDEN };

class Scene {
    SceneState State;
    SceneVisibility Visibility;
public:
    Scene() : State(SCENE_ACTIVE), Visibility(SCENE_VISIBLE) {};
    Scene(const unsigned int& width, const unsigned int& height) : State(SCENE_PAUSE), Visibility(SCENE_HIDDEN), g_windowWidth(width), g_windowHeight(height) {};
    virtual void Init() = 0;
    virtual void Update(float dt) = 0;
    virtual void Render() = 0;

    bool IsPause() const {
        return State == SCENE_PAUSE;
    }

    void SetPause(bool pause) {
        if (pause)
            State = SCENE_PAUSE;
        else
            State = SCENE_ACTIVE;
    }

    bool IsVisible() const {
        return Visibility == SCENE_VISIBLE;
    }

    void SetVisible(bool visible) {
        if (visible) {
            Visibility = SCENE_VISIBLE;
            State = SCENE_ACTIVE;
        }
        else {
            Visibility = SCENE_HIDDEN;
            State = SCENE_PAUSE;
        }
    }

    unsigned int g_windowWidth = 800;
    unsigned int g_windowHeight = 600;
};