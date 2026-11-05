// The interface between the shared screensaver host (common/main.cpp) and one screensaver.
// A screensaver implements Scene and defines CreateScene() and the two strings below.
#pragma once
#include <d2d1.h>
#include <dwrite.h>

class Scene {
 public:
  virtual ~Scene() = default;
  // Set up drawing into hwnd. seed differs per monitor so each screen can do something different.
  virtual bool init(ID2D1Factory* d2d, IDWriteFactory* dw, HWND hwnd, unsigned seed) = 0;
  virtual void resized() = 0;          // the window changed size or DPI
  virtual void frame(double dt) = 0;   // advance by dt seconds and draw one frame
};

Scene* CreateScene();
extern const wchar_t* const kSaverName;   // shown in message boxes; the Screen Saver list uses string 1 in resources.rc
extern const wchar_t* const kSaverAbout;  // shown by the Settings button
