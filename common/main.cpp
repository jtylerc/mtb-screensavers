// The shared screensaver host: the Windows screensaver protocol, a window per monitor, input that ends the
// screensaver, and frame pacing. Each screensaver supplies a Scene (see scene.h) that does the drawing.
//   /s          full screen on every monitor (also the default with no arguments)
//   /p <hwnd>   draw inside the little preview monitor in Screen Saver Settings
//   /c          settings (there are none)
#define _WIN32_WINNT 0x0A00  // Windows 10: per-monitor DPI v2
#include <windows.h>
#include <dwmapi.h>
#include <shellapi.h>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include "scene.h"

namespace {
// set SCREENSAVER_LOG to a file path to trace startup and frame times (for diagnosing a blank or slow screen)
void Log(const char* fmt, ...) {
  static char path[MAX_PATH] = ""; static bool checked = false;
  if (!checked) { checked = true; if (!GetEnvironmentVariableA("SCREENSAVER_LOG", path, MAX_PATH)) path[0] = 0; }
  if (!path[0]) return;
  FILE* f = fopen(path, "a"); if (!f) return;
  va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap); fputc('\n', f); fclose(f);
}
struct Screen { HWND hwnd; std::unique_ptr<Scene> saver; };
std::vector<Screen> g_screens;
bool g_preview = false, g_quit = false;
POINT g_mouse0{-1, -1};

void Quit() { Log("quit"); g_quit = true; PostQuitMessage(0); }

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  switch (msg) {
    case WM_ERASEBKGND: return 1;
    case WM_SIZE:
      for (auto& s : g_screens) if (s.hwnd == hwnd && s.saver) s.saver->resized();
      return 0;
    case WM_DPICHANGED:
      for (auto& s : g_screens) if (s.hwnd == hwnd && s.saver) s.saver->resized();
      return 0;
    case WM_SETCURSOR:
      if (!g_preview) { SetCursor(nullptr); return TRUE; }
      break;
    case WM_KEYDOWN: case WM_SYSKEYDOWN: case WM_LBUTTONDOWN: case WM_RBUTTONDOWN: case WM_MBUTTONDOWN: case WM_MOUSEWHEEL:
      if (!g_preview) Quit();
      break;
    case WM_MOUSEMOVE:
      if (!g_preview) {  // ignore the odd stray event and tiny jitter; a real nudge of the mouse ends it
        POINT p; GetCursorPos(&p);
        if (g_mouse0.x < 0) g_mouse0 = p;
        else if (abs(p.x - g_mouse0.x) + abs(p.y - g_mouse0.y) > 12) Quit();
      }
      break;
    case WM_SYSCOMMAND:
      if (!g_preview && (wp == SC_SCREENSAVE || wp == SC_CLOSE)) return 0;
      break;
    case WM_CLOSE: Log("WM_CLOSE"); Quit(); return 0;
    case WM_DESTROY: Log("WM_DESTROY"); if (g_preview) Quit(); return 0;  // in preview, the settings dialog destroys us with it
  }
  return DefWindowProcW(hwnd, msg, wp, lp);
}

BOOL CALLBACK AddMonitor(HMONITOR, HDC, LPRECT r, LPARAM lp) { reinterpret_cast<std::vector<RECT>*>(lp)->push_back(*r); return TRUE; }
}  // namespace

int WINAPI wWinMain(HINSTANCE inst, HINSTANCE, PWSTR, int) {
  SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);

  // arguments: "/s", "/p 1234", "/p:1234", "/c", "/c:1234" in any case
  int argc = 0; wchar_t** argv = CommandLineToArgvW(GetCommandLineW(), &argc);
  wchar_t mode = L's'; HWND parent = nullptr;
  if (argc > 1) {
    std::wstring a = argv[1];
    size_t k = a.find_first_not_of(L"/-");
    if (k != std::wstring::npos) {
      mode = towlower(a[k]);
      std::wstring rest = a.size() > k + 2 && a[k + 1] == L':' ? a.substr(k + 2) : argc > 2 ? argv[2] : L"";
      parent = (HWND)(INT_PTR)_wtoi64(rest.c_str());
    }
  }
  if (mode == L'c') {
    std::wstring text = std::wstring(kSaverName) + L" has no settings.\n\n" + kSaverAbout, title = std::wstring(kSaverName) + L" screensaver";
    MessageBoxW(parent, text.c_str(), title.c_str(), MB_OK | MB_ICONINFORMATION);
    return 0;
  }
  g_preview = mode == L'p';
  Log("start: mode=%c parent=%p isWindow=%d", (char)mode, (void*)parent, parent ? IsWindow(parent) : -1);
  if (g_preview && !IsWindow(parent)) return 0;

  ID2D1Factory* d2d = nullptr; IDWriteFactory* dw = nullptr;
  if (FAILED(D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, __uuidof(ID2D1Factory), nullptr, (void**)&d2d)) ||
      FAILED(DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory), (IUnknown**)&dw)))
    return 1;

  WNDCLASSW wc{}; wc.lpfnWndProc = WndProc; wc.hInstance = inst; wc.lpszClassName = L"MtbScreensaver";
  wc.hbrBackground = CreateSolidBrush(RGB(0x0d, 0x13, 0x16));
  RegisterClassW(&wc);

  if (g_preview) {
    RECT rc; GetClientRect(parent, &rc);
    g_screens.push_back({CreateWindowExW(0, wc.lpszClassName, L"", WS_CHILD | WS_VISIBLE, 0, 0, rc.right, rc.bottom, parent, nullptr, inst, nullptr), nullptr});
  } else {
    std::vector<RECT> mons; EnumDisplayMonitors(nullptr, nullptr, AddMonitor, (LPARAM)&mons);
    for (const RECT& r : mons)
      g_screens.push_back({CreateWindowExW(WS_EX_TOPMOST | WS_EX_TOOLWINDOW, wc.lpszClassName, kSaverName, WS_POPUP | WS_VISIBLE,
                                           r.left, r.top, r.right - r.left, r.bottom - r.top, nullptr, nullptr, inst, nullptr), nullptr});
  }
  unsigned seed = GetTickCount();
  for (size_t i = 0; i < g_screens.size(); i++) {
    std::unique_ptr<Scene> s(CreateScene());
    bool ok = s->init(d2d, dw, g_screens[i].hwnd, seed + 7919u * (unsigned)i);
    Log("screen %d: hwnd=%p init=%d", (int)i, (void*)g_screens[i].hwnd, ok);
    if (ok) g_screens[i].saver = std::move(s);  // each screen rides its own trail
  }
  if (!g_preview && !g_screens.empty()) { SetForegroundWindow(g_screens[0].hwnd); SetFocus(g_screens[0].hwnd); }

  LARGE_INTEGER freq, last, now; QueryPerformanceFrequency(&freq); QueryPerformanceCounter(&last);
  MSG m;
  while (!g_quit) {
    while (PeekMessageW(&m, nullptr, 0, 0, PM_REMOVE)) {
      if (m.message == WM_QUIT) { g_quit = true; break; }
      TranslateMessage(&m); DispatchMessageW(&m);
    }
    if (g_quit) break;
    QueryPerformanceCounter(&now);
    double dt = double(now.QuadPart - last.QuadPart) / freq.QuadPart; last = now;
    LARGE_INTEGER t0; QueryPerformanceCounter(&t0);
    for (auto& s : g_screens) if (s.saver) s.saver->frame(dt);
    {  // with SCREENSAVER_LOG set, report the average cost of a frame every couple of seconds
      static double acc = 0; static int frames = 0; LARGE_INTEGER t1; QueryPerformanceCounter(&t1);
      acc += double(t1.QuadPart - t0.QuadPart) / freq.QuadPart;
      if (++frames == 120) { Log("%.2f ms/frame over %zu screen(s)", acc / frames * 1000, g_screens.size()); acc = 0; frames = 0; }
    }
    // one wait per frame for the compositor, so every monitor gets a fresh frame each refresh
    if (FAILED(DwmFlush())) Sleep(16);
  }
  Log("exit");
  return 0;
}
