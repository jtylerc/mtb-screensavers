// Forest Ride: the view from the saddle, riding singletrack through a misty forest.
//
// The world is generated along the trail as the rider goes: a winding, rolling trail with roots, rocks and
// leaf litter; firs, bare alders and dead snags; ferns, mossy rocks and fallen logs. Everything is projected
// with a simple pinhole camera and drawn back to front as flat shapes that fade into fog with distance.
// The cockpit (tyre, fork, stem, bar, levers, gloved hands, forearms) is a small 3D model fixed to the bike,
// so the bar steers into turns and the fork compresses over bumps.
//
// Units are metres, seconds and radians. World axes: x east, y up, z north; the trail heading theta is measured
// from +z toward +x, so a positive lateral offset d is to the rider's right.
#define _WIN32_WINNT 0x0A00
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <random>
#include <vector>
#include "canvas.h"
#include "scene.h"

namespace {
const double PI = 3.141592653589793, DEG = PI / 180;
double lerp(double a, double b, double t) { return a + (b - a) * t; }
double clampd(double v, double a, double b) { return std::max(a, std::min(b, v)); }
double smooth(double e0, double e1, double x) { double t = clampd((x - e0) / (e1 - e0), 0, 1); return t * t * (3 - 2 * t); }
Col mix(Col a, Col b, double t) { float k = (float)t; return {a.r + (b.r - a.r) * k, a.g + (b.g - a.g) * k, a.b + (b.b - a.b) * k, a.a + (b.a - a.a) * k}; }
Col alpha(Col c, double a) { c.a = (float)a; return c; }
bool same(const Col& a, const Col& b) { return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a; }
double hash(double n) { double s = std::sin(n * 127.1 + 311.7) * 43758.5453; return s - std::floor(s); }

struct V3 { double x = 0, y = 0, z = 0; };
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
V3 norm(V3 a) { double l = std::sqrt(dot(a, a)); return l > 0 ? a * (1 / l) : a; }
V3 rotAxis(V3 p, V3 k, double ang) {  // rotate p about the unit axis k (Rodrigues)
  double c = std::cos(ang), s = std::sin(ang);
  return p * c + cross(k, p) * s + k * (dot(k, p) * (1 - c));
}

/* ---------- palette: a damp, desaturated forest ---------- */
const Col SKY_TOP = rgba(138, 152, 152), SKY_HORIZON = rgba(204, 210, 203);
const Col FOG = rgba(192, 200, 195), FOG_SUN = rgba(222, 219, 200);
const Col FLOOR = rgba(54, 57, 45);
const Col BARK = rgba(48, 42, 38), BARK_LIT = rgba(92, 82, 70), ALDER = rgba(118, 120, 112), ALDER_LIT = rgba(170, 170, 160), SNAG = rgba(70, 64, 58);
const Col NEEDLES[2] = {rgba(33, 44, 38), rgba(41, 52, 42)};
const Col FERN[2] = {rgba(66, 82, 54), rgba(52, 66, 45)};
const Col DIRT = rgba(102, 88, 70), DIRT_EDGE = rgba(74, 66, 52), DIRT_WORN = rgba(118, 104, 84);
const Col LITTER[3] = {rgba(132, 110, 80), rgba(66, 56, 44), rgba(92, 94, 72)};
const Col ROOT = rgba(56, 43, 34), ROCK = rgba(84, 86, 80), ROCK_TOP = rgba(112, 114, 106), MOSS = rgba(70, 86, 54);
const Col LOG = rgba(66, 58, 50), PATCH[2] = {rgba(60, 64, 49), rgba(49, 51, 41)};
// the cockpit, close up and unfogged
const Col BLACK = rgba(24, 25, 27), DARK = rgba(36, 37, 40), RIM = rgba(150, 157, 154), FRAME = rgba(52, 62, 64), FRAME_HI = rgba(120, 136, 136);
const Col GOLD = rgba(150, 128, 82), GOLD_HI = rgba(214, 196, 150), TIRE = rgba(20, 20, 20), KNOB = rgba(34, 34, 33);
const Col GLOVE = rgba(40, 42, 44), GLOVE_HI = rgba(104, 108, 106), CUFF = rgba(118, 136, 124), SLEEVE = rgba(46, 60, 62), SLEEVE_HI = rgba(104, 122, 120);

const double STEP = 0.5;      // trail sample spacing
const double CHUNK = 2.0;     // world generation granularity along the trail
const double AHEAD = 115;     // how far ahead things are generated and drawn
const double NEAR_Z = 0.12;     // camera near plane
const double EYE = 1.55;      // eye height above the trail
const double LOOK_DOWN = 17 * DEG;  // the rider looks a little down the trail
}  // namespace

class Forest : public Scene {
 public:
  bool init(ID2D1Factory* f, IDWriteFactory* dw, HWND hwnd, unsigned seed) override;
  void resized() override;
  void frame(double dt) override;

 private:
  struct TP { double s, x, z; };
  struct Tree { double s, d, r, h, crown; int kind; double k; };
  struct Fern { double s, d, size, rot; int c; };
  struct Speck { double s, d, size, rot; int c; };
  struct Root { double s, ang, w, len; };
  struct Rock { double s, d, r; bool trail; };
  struct Log { double s, d, len, r, ang; };
  struct Patch { double s, d, rx, rz, rot; int c; };
  struct Bump { double s, h, w; };
  struct Mote { V3 p; double r, ph; };
  // A shape ready to fill. Drawn far band to near band; within a band, flat things (layer 0, 1) before upright
  // ones (2+), then grouped by colour so each group is one Direct2D fill.
  struct Prim { float depth; int band; int layer; uint32_t key; Col col; uint32_t start, count; };

  double rnd() { return std::uniform_real_distribution<double>(0, 1)(rng_); }
  // trail shape: analytic heading and height along the distance s
  double theta(double s) const;
  double curvature(double s) const;
  double height(double s) const;
  double slope(double s) const;
  double halfWidth(double s) const { return 0.43 + 0.07 * std::sin(s / 9.3 + ph_[9]); }
  void extendTrail(double to);
  V3 at(double s, double d) const;  // world point on the ground at trail distance s, lateral offset d
  void generate(double to);
  double bumpAt(double s) const;
  // camera
  V3 toCam(V3 p) const { V3 d = p - eye_; return {dot(d, right_), dot(d, up_), dot(d, fwd_)}; }
  V2 proj(V3 c) const { return {cx_ + F_ * c.x / c.z, cy_ - F_ * c.y / c.z}; }
  Col fogged(Col base, double dist) const;
  static constexpr int BANDS = 24;  // fog steps for objects; each step is one colour per material, so fewer steps mean fewer fills
  int bandOf(double depth) const { return (int)std::round((1 - std::exp(-std::max(0.0, depth - 1.5) / 26)) * BANDS); }
  void addPoly(const V3* cam, int n, double depth, Col col, int layer);  // camera-space polygon, clipped to the near plane
  void addWorld(std::initializer_list<V3> pts, double depth, Col col, int layer);
  // drawing
  void drawSky();
  void drawGround();  // the forest floor and the trail
  double fogAt(double dist) const;
  // fill the current path with base fogged by distance; ref(d) is the world point d metres ahead whose
  // screen position anchors that distance (on the trail for the trail, on the ground plane for the ground)
  template <class Ref> void fillByDistance(Col base, double maxDist, Ref ref);
  double groundPlane(double ahead) const;
  void buildWorld();
  void addTree(const Tree& t);
  void drawPrims();
  void drawMotes(double dt);
  void drawCockpit();
  // cockpit helpers (bike space -> screen)
  V3 bikeToCam(V3 b) const;
  bool bikeProj(V3 b, V2& out, double& z) const;
  void tube(const std::vector<V3>& pts, const std::vector<double>& radii, Col col, Col hi, double hiAlpha = 0.55);
  void blob(const std::vector<V3>& pts, Col col);

  Canvas ctx; HWND hwnd_ = nullptr; std::mt19937 rng_; unsigned seed_ = 0;
  double W = 0, H = 0, dpr = 1, F_ = 1, cx_ = 0, cy_ = 0;
  double ph_[16] = {};
  std::deque<TP> trail_;
  double genTo_ = 0;
  std::deque<Tree> trees_; std::deque<Fern> ferns_; std::deque<Speck> specks_; std::deque<Root> roots_;
  std::deque<Rock> rocks_; std::deque<Log> logs_; std::deque<Patch> patches_; std::deque<Bump> bumps_;
  std::vector<Mote> motes_;
  // ride state
  double s_ = 0, v_ = 5, time_ = 0, lat_ = 0, latV_ = 0, yaw_ = 0, roll_ = 0, pitch_ = 0, steer_ = 0;
  double barY_ = 0, barV_ = 0, eyeY_ = 0, eyeV_ = 0, rearY_ = 0, comp_ = 0.045, wheel_ = 0, sunAz_ = 0, sunFacing_ = 0;
  V3 eye_, right_, up_, fwd_;
  Col fog_{};
  std::vector<V2> pool_; std::vector<Prim> prims_;
};

/* ---------- setup ---------- */
Scene* CreateScene() { return new Forest; }
const wchar_t* const kSaverName = L"Forest Ride";
const wchar_t* const kSaverAbout = L"The view from the saddle, riding singletrack through a misty forest.";

bool Forest::init(ID2D1Factory* f, IDWriteFactory* dw, HWND hwnd, unsigned seed) {
  hwnd_ = hwnd; seed_ = seed; rng_.seed(seed);
  if (!ctx.init(f, dw, hwnd)) return false;
  for (double& p : ph_) p = rnd() * 2 * PI;
  sunAz_ = rnd() * 2 * PI;
  s_ = 20;  // start a little way in so there is trail behind the camera
  extendTrail(s_ + AHEAD + 80);
  genTo_ = std::floor((s_ - 8) / CHUNK) * CHUNK;
  generate(s_ + AHEAD);
  for (int i = 0; i < 60; i++) motes_.push_back({{rnd() * 8 - 4, rnd() * 3 - 1.2, 0.5 + rnd() * 9}, 0.6 + rnd() * 1.2, rnd() * 6.28});
  yaw_ = theta(s_ + 4); barY_ = eyeY_ = rearY_ = bumpAt(s_);
  resized();
  return true;
}

void Forest::resized() {
  RECT rc; GetClientRect(hwnd_, &rc); ctx.resize(rc.right, rc.bottom);
  dpr = GetDpiForWindow(hwnd_) / 96.0; if (dpr <= 0) dpr = 1;
  W = rc.right / dpr; H = rc.bottom / dpr;
  // a wide, helmet-camera field of view: 100° across on landscape screens, a little narrower on portrait ones
  F_ = (W / 2) / std::tan((W >= H ? 50 : 40) * DEG);
  cx_ = W / 2; cy_ = H / 2;
}

/* ---------- the trail ---------- */
// heading wanders through incommensurate sines so the trail never repeats; turns are no tighter than ~7 m radius
double Forest::theta(double s) const {
  return 0.85 * std::sin(s / 19 + ph_[0]) + 0.5 * std::sin(s / 8.7 + ph_[1]) + 0.18 * std::sin(s / 4.3 + ph_[2]) + 0.9 * std::sin(s / 97 + ph_[3]);
}
double Forest::curvature(double s) const {
  return 0.85 / 19 * std::cos(s / 19 + ph_[0]) + 0.5 / 8.7 * std::cos(s / 8.7 + ph_[1]) + 0.18 / 4.3 * std::cos(s / 4.3 + ph_[2]) + 0.9 / 97 * std::cos(s / 97 + ph_[3]);
}
// a gentle descent with rolling dips and rises (kept modest: see drawGround)
double Forest::height(double s) const {
  return -0.025 * s + 1.4 * std::sin(s / 33 + ph_[4]) + 0.55 * std::sin(s / 13.5 + ph_[5]) + 0.18 * std::sin(s / 5.6 + ph_[6]);
}
double Forest::slope(double s) const {
  return -0.025 + 1.4 / 33 * std::cos(s / 33 + ph_[4]) + 0.55 / 13.5 * std::cos(s / 13.5 + ph_[5]) + 0.18 / 5.6 * std::cos(s / 5.6 + ph_[6]);
}
void Forest::extendTrail(double to) {
  if (trail_.empty()) trail_.push_back({0, 0, 0});
  while (trail_.back().s < to) {
    const TP& b = trail_.back(); double th = theta(b.s + STEP / 2);
    trail_.push_back({b.s + STEP, b.x + std::sin(th) * STEP, b.z + std::cos(th) * STEP});
  }
  while (trail_.size() > 2 && trail_[1].s < s_ - 90) trail_.pop_front();
}
V3 Forest::at(double s, double d) const {
  double f = (s - trail_.front().s) / STEP; size_t i = (size_t)clampd(std::floor(f), 0, (double)trail_.size() - 2); f = clampd(f - i, 0, 1.5);
  const TP &a = trail_[i], &b = trail_[i + 1];
  double th = theta(s), x = lerp(a.x, b.x, f) + std::cos(th) * d, z = lerp(a.z, b.z, f) - std::sin(th) * d;
  // off the trail the ground undulates a little (no side slope: the forest floor drawn behind everything is
  // shaded from the trail's own height profile, so the terrain mustn't stray far from it)
  double hw = halfWidth(s);
  double y = height(s) + (std::fabs(d) > hw + 0.4 ? 0.25 * std::sin(d / 3.1 + s / 7.7) * smooth(0.5, 4, std::fabs(d) - hw) : 0);
  return {x, y, z};
}

/* ---------- the forest, generated in 2 m chunks along the trail ---------- */
void Forest::generate(double to) {
  while (genTo_ < to) {
    double s0 = genTo_; genTo_ += CHUNK;
    std::mt19937 r(seed_ * 7919u + (unsigned)(int64_t)std::llround(s0 / CHUNK) * 104729u);
    auto u = [&] { return std::uniform_real_distribution<double>(0, 1)(r); };
    // keep things off the trail where it loops back near itself
    auto clearOfTrail = [&](double s, double d, double margin) {
      V3 p = at(s, d);
      for (double q = s - 70; q <= s + 70; q += 1.5) {
        if (std::fabs(q - s) < 2) continue;
        V3 c = at(q, 0); double dx = p.x - c.x, dz = p.z - c.z;
        if (dx * dx + dz * dz < margin * margin) return false;
      }
      return true;
    };
    for (int side = -1; side <= 1; side += 2) {
      for (int k = 0; k < 3; k++) {
        double s = s0 + u() * CHUNK, d = side * (1.35 + std::pow(u(), 1.15) * 44);
        double rr = u(), kind = u();
        Tree t{s, d, 0.11 + rr * rr * 0.42, 0, 0, kind < 0.72 ? 0 : kind < 0.9 ? 1 : 2, u() * 100};
        t.h = t.kind == 2 ? 5 + u() * 9 : 17 + u() * 22 + t.r * 12;
        t.crown = t.kind == 0 ? 2 + u() * 4 + t.r * 9 : 3 + u() * 5;
        if (clearOfTrail(s, d, 1.0 + t.r)) trees_.push_back(t);
      }
      for (int k = 0; k < 3; k++) {
        double s = s0 + u() * CHUNK, d = side * (halfWidth(s) + 0.12 + std::pow(u(), 2) * 7);
        if (clearOfTrail(s, d, 0.9)) ferns_.push_back({s, d, 0.3 + u() * 0.45, u() * 6.28, (int)(u() * 2)});
      }
      for (int k = 0; k < 2; k++) {
        double s = s0 + u() * CHUNK, d = side * (0.9 + u() * 24);
        patches_.push_back({s, d, 0.4 + u() * 1.1, 0.3 + u() * 0.7, u() * 3, 0});
      }
    }
    for (int k = 0; k < 14; k++) {  // needles, leaves and grit on the trail
      double s = s0 + u() * CHUNK; specks_.push_back({s, (u() * 2 - 1) * halfWidth(s) * 0.95, 0.018 + u() * 0.05, u() * 3.14, (int)(u() * 3)});
    }
    if (u() < 0.32 && s0 > 30) {  // a root across the trail
      double s = s0 + u() * CHUNK; Root rt{s, (u() - 0.5) * 1.1, 0.035 + u() * 0.04, 0.8 + u() * 0.6};
      roots_.push_back(rt); bumps_.push_back({s, rt.w * 0.9, 0.12});
    }
    if (u() < 0.14 && s0 > 30) {  // a rock in the trail
      double s = s0 + u() * CHUNK, rr = 0.05 + u() * 0.08; rocks_.push_back({s, (u() * 2 - 1) * 0.3, rr, true});
      bumps_.push_back({s, rr * 0.8, 0.12 + rr});
    }
    if (u() < 0.35) {  // mossy rocks beside it
      double side = u() < 0.5 ? -1 : 1, s = s0 + u() * CHUNK, d = side * (0.9 + std::pow(u(), 1.5) * 16);
      if (clearOfTrail(s, d, 1.0)) rocks_.push_back({s, d, 0.15 + u() * u() * 0.45, false});
    }
    if (u() < 0.05) {  // a fallen log
      double side = u() < 0.5 ? -1 : 1, s = s0 + u() * CHUNK, d = side * (2.2 + u() * 9), len = 3 + u() * 5;
      if (clearOfTrail(s, d, len / 2 + 1)) logs_.push_back({s, d, len, 0.16 + u() * 0.2, (u() - 0.5) * 0.7});
    }
  }
  auto behind = [&](double s) { return s < s_ - 8; };
  while (!trees_.empty() && behind(trees_.front().s + 6)) trees_.pop_front();
  while (!ferns_.empty() && behind(ferns_.front().s)) ferns_.pop_front();
  while (!specks_.empty() && behind(specks_.front().s)) specks_.pop_front();
  while (!roots_.empty() && behind(roots_.front().s)) roots_.pop_front();
  while (!rocks_.empty() && behind(rocks_.front().s)) rocks_.pop_front();
  while (!logs_.empty() && behind(logs_.front().s + 8)) logs_.pop_front();
  while (!patches_.empty() && behind(patches_.front().s + 3)) patches_.pop_front();
  while (!bumps_.empty() && behind(bumps_.front().s)) bumps_.pop_front();
}

// how much a wheel at trail distance s is lifted by roots and rocks (plus a little fine roughness)
double Forest::bumpAt(double s) const {
  double h = 0.006 * std::sin(s * 3.1 + ph_[8]) + 0.004 * std::sin(s * 7.3);
  for (const Bump& b : bumps_) {
    double x = (s - b.s) / b.w;
    if (x > -1 && x < 1) h = std::max(h, b.h * (1 - x * x));
  }
  return h;
}

/* ---------- camera and projection ---------- */
Col Forest::fogged(Col base, double dist) const {
  double f = 1 - std::exp(-std::max(0.0, dist - 1.5) / 26);
  f = std::round(f * BANDS) / BANDS;  // quantized so nearby shapes share a colour and can be filled together
  return mix(base, fog_, f);
}

void Forest::addPoly(const V3* cam, int n, double depth, Col col, int layer) {
  // clip to the near plane (Sutherland-Hodgman against z = NEAR_Z)
  V3 buf[160]; int m = 0;
  for (int i = 0; i < n && m < 158; i++) {
    const V3 &a = cam[i], &b = cam[(i + 1) % n];
    bool ia = a.z >= NEAR_Z, ib = b.z >= NEAR_Z;
    if (ia) buf[m++] = a;
    if (ia != ib) { double t = (NEAR_Z - a.z) / (b.z - a.z); buf[m++] = a + (b - a) * t; }
  }
  if (m < 3) return;
  if (layer < 9 && bandOf(depth) >= BANDS - 1) return;  // lost in the fog
  uint32_t start = (uint32_t)pool_.size();
  double area = 0, minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
  for (int i = 0; i < m; i++) {
    V2 p = proj(buf[i]); pool_.push_back(p);
    minx = std::min(minx, p.x); maxx = std::max(maxx, p.x); miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
  }
  if (maxx < -2 || minx > W + 2 || maxy < -2 || miny > H + 2) { pool_.resize(start); return; }  // off screen
  for (int i = 0; i < m; i++) { const V2 &a = pool_[start + i], &b = pool_[start + (i + 1) % m]; area += a.x * b.y - b.x * a.y; }
  if (area < 0) std::reverse(pool_.begin() + start, pool_.end());  // one winding, so overlapping shapes in a batch don't punch holes
  uint32_t key = (uint32_t)(col.r * 255) << 24 | (uint32_t)(col.g * 255) << 16 | (uint32_t)(col.b * 255) << 8 | (uint32_t)(col.a * 255);
  prims_.push_back({(float)depth, bandOf(depth), layer, key, col, start, (uint32_t)m});
}

void Forest::addWorld(std::initializer_list<V3> pts, double depth, Col col, int layer) {
  V3 cam[16]; int n = 0;
  for (const V3& p : pts) if (n < 16) cam[n++] = toCam(p);
  addPoly(cam, n, depth, col, layer);
}

/* ---------- sky and ground ---------- */
void Forest::drawSky() {
  // gradient along the camera's view of "up", so it tilts with the horizon when the rider leans
  V3 hz = toCam(eye_ + V3{fwd_.x, 0, fwd_.z} * 1000), top = toCam(eye_ + V3{fwd_.x * 0.5, 1, fwd_.z * 0.5} * 1000);
  V2 a = hz.z > 0 ? proj(hz) : V2{cx_, cy_}, b = top.z > 0 ? proj(top) : V2{cx_, -H};
  ctx.beginPath(); ctx.rect(-4, -4, W + 8, H + 8);
  ctx.fillLinear(a, a + (b - a) * 0.6, {{0, mix(SKY_HORIZON, FOG_SUN, sunFacing_ * 0.5)}, {1, SKY_TOP}});
  // the low sun glowing through the mist
  V3 sun{std::sin(sunAz_) * std::cos(28 * DEG), std::sin(28 * DEG), std::cos(sunAz_) * std::cos(28 * DEG)};
  V3 sc = toCam(eye_ + sun * 1000);
  if (sc.z > 0) {
    V2 p = proj(sc); double r = H * 0.9;
    ctx.beginPath(); ctx.rect(-4, -4, W + 8, H + 8);
    ctx.fillRadial(p, r, r, {{0, rgba(255, 248, 226, 0.55 * sunFacing_)}, {0.25, rgba(250, 240, 214, 0.22 * sunFacing_)}, {1, rgba(240, 232, 210, 0)}});
  }
}

double Forest::fogAt(double dist) const { return 1 - std::exp(-std::max(0.0, dist - 1.5) / 26); }

// Fill the current path with a colour that fogs smoothly with distance along the ground. The gradient runs down
// the screen from the horizon; its stops come from projecting points on the trail at known distances.
// the ground plane ahead of the rider: follows the trail's grade for the first 60 m, then levels out
double Forest::groundPlane(double ahead) const {
  double gr = slope(s_ + 4), a = std::max(0.0, ahead);
  return height(s_) + gr * std::min(60.0, a) - std::max(0.0, a - 60) * 0.004;
}

template <class Ref> void Forest::fillByDistance(Col base, double maxDist, Ref ref) {
  V3 fh = norm(V3{fwd_.x, 0, fwd_.z}), rh{fh.z, 0, -fh.x};
  V3 hA = toCam(eye_ + fh * 3000 - rh * 3000), hB = toCam(eye_ + fh * 3000 + rh * 3000);
  if (hA.z <= 0 || hB.z <= 0) { ctx.fillStyle = fogged(base, maxDist * 0.5); ctx.fill(); return; }
  V2 a = proj(hA), b = proj(hB), dir = b - a; double l = std::hypot(dir.x, dir.y); if (l < 1) l = 1;
  V2 down{-dir.y / l, dir.x / l}; if (down.y < 0) down = down * -1.0;
  V2 p0 = proj(toCam(eye_ + fh * 3000)); double span = H * 1.6;
  std::vector<Stop> stops;
  double last = -1;
  const double dists[] = {3000, 400, 160, 100, 70, 50, 38, 29, 22, 17, 13, 10, 7.5, 5.5, 4, 3, 2.2, 1.6, 1.1, 0.7};
  for (double d : dists) {
    if (d > maxDist * 1.5 && d != 3000) continue;
    V3 c = toCam(ref(std::min(d, maxDist)));
    if (c.z < NEAR_Z) break;
    V2 q = proj(c); double t = ((q.x - p0.x) * down.x + (q.y - p0.y) * down.y) / span;
    if (t <= last + 1e-4) continue;
    if (t > 1) { stops.push_back({1.0f, mix(base, fog_, fogAt(d))}); last = 1; break; }
    stops.push_back({(float)std::max(0.0, t), mix(base, fog_, fogAt(d))}); last = t;
  }
  if (last < 1) stops.push_back({1.0f, mix(base, fog_, fogAt(0.5))});
  if (stops.front().pos > 0) stops.insert(stops.begin(), {0.0f, stops.front().col});
  if (stops.size() < 2) { ctx.fillStyle = fogged(base, 10); ctx.fill(); return; }
  ctx.fillLinear(p0, p0 + down * span, stops);
}

void Forest::drawGround() {
  // The forest floor behind everything: a flat disc at the rider's level. Its shading comes from the real
  // terrain ahead: each distance's fog sits where the trail at that distance appears on screen, and the
  // disc's own horizon is full fog, so the floor under far-off trees matches them going up or downhill.
  V3 fh = norm(V3{fwd_.x, 0, fwd_.z}), rh{fh.z, 0, -fh.x}, foot{eye_.x, height(s_), eye_.z};
  V3 cam[40]; int n = 0;
  for (int k = 0; k < 36; k++) {  // the near plane trims away what's behind the rider
    double a = yaw_ + k * 10 * DEG; cam[n++] = toCam(foot + V3{std::sin(a), 0, std::cos(a)} * 3000);
  }
  uint32_t before = (uint32_t)prims_.size();
  addPoly(cam, n, 0, FLOOR, 0);
  if (prims_.size() == before) return;
  const Prim& pr = prims_.back(); ctx.beginPath();
  for (uint32_t k = 0; k < pr.count; k++) { if (k) ctx.lineTo(pool_[pr.start + k]); else ctx.moveTo(pool_[pr.start + k]); }
  ctx.closePath(); pool_.resize(pr.start); prims_.pop_back();
  V3 hA = toCam(eye_ + fh * 3000 - rh * 3000), hB = toCam(eye_ + fh * 3000 + rh * 3000), h0 = toCam(eye_ + fh * 3000);
  if (hA.z <= 0 || hB.z <= 0 || h0.z <= 0) { ctx.fillStyle = mix(FLOOR, fog_, 0.5); ctx.fill(); return; }
  V2 a = proj(hA), b = proj(hB), dir = b - a; double l = std::max(1.0, std::hypot(dir.x, dir.y));
  V2 down{-dir.y / l, dir.x / l}; if (down.y < 0) down = down * -1.0;
  V2 p0 = proj(h0); double span = H * 1.6;
  std::vector<std::pair<double, double>> marks{{0.0, 1.0}};  // (position down the screen, fog)
  const double dists[] = {110, 80, 60, 45, 34, 26, 20, 15, 11, 8, 6, 4.5, 3.2, 2.2, 1.5, 1.0};
  for (double d : dists) {
    V3 c = toCam(at(s_ + d, 0)); if (c.z < NEAR_Z) continue;
    V2 q = proj(c); marks.push_back({((q.x - p0.x) * down.x + (q.y - p0.y) * down.y) / span, fogAt(d)});
  }
  std::sort(marks.begin(), marks.end());
  std::vector<Stop> stops; double lastT = -1, lastF = 2;
  for (auto& [t, f] : marks) {  // keep it consistent: further down the screen is never foggier
    if (t <= lastT + 1e-4 || f > lastF || t < 0 || t > 1) continue;
    stops.push_back({(float)t, mix(FLOOR, fog_, f)}); lastT = t; lastF = f;
  }
  if (stops.empty() || stops.front().pos > 0) stops.insert(stops.begin(), {0.0f, fog_});
  if (stops.back().pos < 1) stops.push_back({1.0f, mix(FLOOR, fog_, std::min(lastF, fogAt(1.0)))});
  ctx.fillLinear(p0, p0 + down * span, stops);

  // the trail: worn dirt between darker edges. It is drawn in short sections, far to near, each filled with
  // a gradient along its own length (a single gradient can't follow the trail round a bend). Each section
  // reaches a little into the farther one, where their colours match, so no seam shows.
  auto section = [&](double sa, double sb, double maxAhead, auto left, auto right, Col base) {
    if (sa - s_ > maxAhead) return;
    sb = std::min(sb, s_ + maxAhead);
    std::vector<V3> L, R; int steps = std::max(2, (int)std::ceil((sb - sa) / 0.5));
    for (int k = 0; k <= steps; k++) { double ss = lerp(sa, sb, (double)k / steps); L.push_back(toCam(at(ss, left(ss)))); R.push_back(toCam(at(ss, right(ss)))); }
    ctx.beginPath(); bool started = false;
    auto put = [&](V3 c) { V2 q = proj(c); if (started) ctx.lineTo(q); else { ctx.moveTo(q); started = true; } };
    auto side = [&](const std::vector<V3>& P, bool fwd) {
      size_t n = P.size();
      for (size_t k = 0; k < n; k++) {
        size_t i = fwd ? k : n - 1 - k, prev = fwd ? i - 1 : i + 1;
        bool in = P[i].z >= NEAR_Z;
        if (k > 0) { const V3& pp = P[prev]; bool pin = pp.z >= NEAR_Z; if (in != pin) { double t = (NEAR_Z - pp.z) / (P[i].z - pp.z); put(pp + (P[i] - pp) * t); } }
        if (in) put(P[i]);
      }
    };
    side(L, true); side(R, false);
    if (!started) return;
    ctx.closePath();
    // gradient from this section's near end to its far end, along the trail's centre on screen
    V3 cn = toCam(at(std::max(sa, s_ + 0.6), 0)), cf = toCam(at(sb, 0));
    double dn = std::max(0.6, sa - s_), df = sb - s_;
    if (cn.z < NEAR_Z || cf.z < NEAR_Z) { ctx.fillStyle = mix(base, fog_, fogAt((dn + df) / 2)); ctx.fill(); return; }
    V2 pn = proj(cn), pf = proj(cf);
    if (std::hypot(pf.x - pn.x, pf.y - pn.y) < 2) { ctx.fillStyle = mix(base, fog_, fogAt((dn + df) / 2)); ctx.fill(); return; }
    ctx.fillLinear(pn, pf, {{0, mix(base, fog_, fogAt(dn))}, {1, mix(base, fog_, fogAt(df))}});
  };
  auto hwL = [&](double s) { return -halfWidth(s); };
  auto hwR = [&](double s) { return halfWidth(s); };
  auto edgeLo = [&](double s) { return -halfWidth(s) - 0.12; };
  auto edgeLi = [&](double s) { return -halfWidth(s) * 0.7; };
  auto edgeRi = [&](double s) { return halfWidth(s) * 0.7; };
  auto edgeRo = [&](double s) { return halfWidth(s) + 0.12; };
  auto wornL = [&](double s) { return -(0.12 + 0.05 * std::sin(s * 0.7)); };
  auto wornR = [&](double s) { return 0.12 + 0.05 * std::sin(s * 0.7); };
  // section boundaries: short near the rider, longer further off
  std::vector<double> cuts;
  for (double a = 0; a < AHEAD;) { cuts.push_back(a); a += a < 12 ? 3 : a < 30 ? 6 : a < 60 ? 10 : 18; }
  cuts.push_back(AHEAD);
  for (size_t k = cuts.size() - 1; k-- > 0;) {
    double sa = s_ + (k == 0 ? -1.0 : cuts[k]), sb = s_ + cuts[k + 1] + (k + 2 < cuts.size() ? 0.4 : 0);
    section(sa, sb, AHEAD, hwL, hwR, DIRT);
    section(sa, sb, 60, edgeLo, edgeLi, DIRT_EDGE);
    section(sa, sb, 60, edgeRi, edgeRo, DIRT_EDGE);
    section(sa, sb, 60, wornL, wornR, DIRT_WORN);
  }
}

/* ---------- the forest ---------- */
void Forest::addTree(const Tree& t) {
  V3 base = at(t.s, t.d); base.y -= 0.1;
  V3 bc = toCam(base);
  double wide = t.kind == 0 ? 2.2 + t.r * 5 : 1.5;
  if (bc.z < NEAR_Z || bc.z > AHEAD + 10 || std::fabs(bc.x) > bc.z * 1.4 + wide + 1) return;
  double dist = std::sqrt(dot(base - eye_, base - eye_));
  V3 R = norm(V3{fwd_.z, 0, -fwd_.x}), U{0, 1, 0};  // billboard: faces the camera, stays upright in the world
  auto P = [&](double x, double y) { return base + R * x + U * y; };
  double r = t.r, h = t.h, depth = bc.z;
  Col bark = t.kind == 1 ? ALDER : t.kind == 2 ? SNAG : BARK, lit = t.kind == 1 ? ALDER_LIT : BARK_LIT;
  // trunk, tapering, with a little lean
  double lean = (hash(t.k) - 0.5) * 0.06 * h;
  addWorld({P(-r, 0), P(r, 0), P(r * 0.3 + lean, h), P(-r * 0.3 + lean, h)}, depth, fogged(bark, dist), 2);
  if (dist < 22) {  // the lit side of the trunk
    double side = std::sin(sunAz_ - yaw_) > 0 ? 1 : -1;
    addWorld({P(side * r * 0.45, 0), P(side * r, 0), P(side * r * 0.3 + lean, h), P(side * r * 0.12 + lean, h)}, depth - 0.001, fogged(mix(bark, lit, 0.45), dist), 3);
  }
  if (t.kind == 0) {  // a fir: a ragged cone of drooping branch tiers
    int n = dist > 50 ? 7 : std::clamp((int)((h - t.crown) / 1.0), 8, 26);
    double y0 = t.crown, span = h - y0, wmax = 1.0 + t.r * 4.2;
    V3 cam[2 * 27 * 2 + 4]; int m = 0;
    std::vector<V3> left, rightSide;
    for (int i = 0; i <= n; i++) {
      double y = y0 + span * i / n, f = 1 - (double)i / n, w = wmax * std::pow(f, 0.85) * (0.75 + 0.5 * hash(t.k + i * 1.7));
      double droop = 0.25 + 0.35 * f;
      double lx = lean * y / h;
      left.push_back(P(-w + lx, y - droop)); left.push_back(P(-w * 0.45 + lx, y + span / n * 0.45));
      rightSide.push_back(P(w + lx, y - droop * (0.8 + 0.4 * hash(t.k + i))));
      rightSide.push_back(P(w * 0.45 + lx, y + span / n * 0.45));
    }
    for (auto it = left.begin(); it != left.end(); ++it) cam[m++] = toCam(*it);
    cam[m++] = toCam(P(lean, h + 0.6));
    for (auto it = rightSide.rbegin(); it != rightSide.rend(); ++it) cam[m++] = toCam(*it);
    addPoly(cam, m, depth - 0.002, fogged(NEEDLES[(int)(hash(t.k) * 2)], dist), 4);
  } else {  // an alder or a snag: a few bare branches reaching up
    int nb = t.kind == 1 ? 5 : 2;
    for (int i = 0; i < nb; i++) {
      double y = t.crown + (h - t.crown) * (0.1 + 0.8 * hash(t.k + i * 3.3)), side = (i % 2 ? 1 : -1);
      double len = (t.kind == 1 ? 1.6 + 2.2 * hash(t.k + i) : 0.6 + hash(t.k + i)) * (1 - 0.5 * (y - t.crown) / (h - t.crown + 0.01));
      double w = std::max(0.025, r * 0.32 * (1 - y / h)), lx = lean * y / h;
      addWorld({P(lx - w * side * 0.2, y - w), P(lx + side * len, y + len * (t.kind == 1 ? 0.8 : 0.15)), P(lx + side * len * 0.95, y + len * (t.kind == 1 ? 0.8 : 0.15) + w), P(lx, y + w * 2)},
               depth - 0.002, fogged(bark, dist), 4);
    }
  }
}

void Forest::buildWorld() {
  for (const Patch& p : patches_) {
    if (p.s - s_ > 14) break;
    V3 c = at(p.s, p.d); double depth = toCam(c).z; if (depth < NEAR_Z) continue;
    V3 cam[10];
    for (int k = 0; k < 10; k++) {
      double a = k * PI / 5 + p.rot, rr = 1 - 0.25 * hash(p.s + k);
      cam[k] = toCam(c + V3{std::cos(a) * p.rx * rr, 0.01, std::sin(a) * p.rz * rr});
    }
    addPoly(cam, 10, depth + 0.5, mix(PATCH[p.c], fog_, fogAt(depth)), 0);
  }
  for (const Speck& sp : specks_) {
    if (sp.s - s_ > 16) break;
    V3 c = at(sp.s, sp.d); c.y += 0.012; double depth = toCam(c).z; if (depth < NEAR_Z) continue;
    double ca = std::cos(sp.rot) * sp.size, sa = std::sin(sp.rot) * sp.size;
    addWorld({c + V3{ca, 0, sa}, c + V3{-sa * 0.35, 0, ca * 0.35}, c + V3{-ca, 0, -sa}, c + V3{sa * 0.35, 0, -ca * 0.35}}, depth, fogged(LITTER[sp.c], depth), 1);
  }
  for (const Root& rt : roots_) {
    if (rt.s - s_ > 45) break;
    double depth = toCam(at(rt.s, 0)).z; if (depth < NEAR_Z) continue;
    V3 top[7], bot[7];
    for (int k = 0; k < 7; k++) {
      double t = (k / 6.0 - 0.5) * 2 * rt.len, ss = rt.s + t * std::sin(rt.ang) + 0.06 * std::sin(t * 4 + rt.s);
      V3 p = at(ss, t * std::cos(rt.ang)); double lift = rt.w * 0.5 * (1 - std::pow(std::fabs(t) / rt.len, 3));
      top[k] = p + V3{0, lift + rt.w * 0.3, 0}; bot[k] = at(ss - rt.w, t * std::cos(rt.ang)) + V3{0, 0.005, 0};
    }
    V3 cam[14]; int n = 0;
    for (int k = 0; k < 7; k++) cam[n++] = toCam(top[k]);
    for (int k = 6; k >= 0; k--) cam[n++] = toCam(bot[k]);
    addPoly(cam, n, depth - 0.01, fogged(ROOT, depth), 1);
  }
  for (const Rock& rk : rocks_) {
    if (rk.s - s_ > AHEAD) break;
    V3 c = at(rk.s, rk.d); double depth = toCam(c).z; if (depth < NEAR_Z) continue;
    V3 R = norm(V3{fwd_.z, 0, -fwd_.x}), cam[12];
    for (int k = 0; k < 9; k++) {  // a rounded hump, billboarded
      double a = PI * k / 8, rr = rk.r * (0.8 + 0.35 * hash(rk.s * 3 + k));
      cam[k] = toCam(c + R * (std::cos(a) * rr * 1.2) + V3{0, std::sin(a) * rr * 0.75, 0});
    }
    addPoly(cam, 9, depth - 0.01, fogged(ROCK, depth), 2);
    for (int k = 0; k < 7; k++) {
      double a = PI * (0.18 + 0.64 * k / 6), rr = rk.r * (0.8 + 0.35 * hash(rk.s * 3 + k + 1));
      cam[k] = toCam(c + R * (std::cos(a) * rr * 1.05) + V3{0, std::sin(a) * rr * 0.72, 0});
    }
    cam[7] = toCam(c + R * (rk.r * 0.6) + V3{0, rk.r * 0.42, 0}); cam[8] = toCam(c + R * (-rk.r * 0.6) + V3{0, rk.r * 0.4, 0});
    addPoly(cam, 9, depth - 0.011, fogged(rk.trail ? ROCK_TOP : MOSS, depth), 3);
    (void)0;
  }
  for (const Log& lg : logs_) {
    if (lg.s - s_ > AHEAD) break;
    V3 c = at(lg.s, lg.d); double depth = toCam(c).z; if (depth < 0.5) continue;
    double th = theta(lg.s) + lg.ang; V3 dir{std::sin(th), 0, std::cos(th)};
    V3 a = c - dir * (lg.len / 2) + V3{0, lg.r * 0.8, 0}, b = c + dir * (lg.len / 2) + V3{0, lg.r * 0.8, 0};
    // seen from anywhere a log is a capsule: the two end circles joined by their outer tangents
    V3 ca = toCam(a), cb = toCam(b);
    if (ca.z < 0.5 || cb.z < 0.5) continue;
    V2 pa = proj(ca), pb = proj(cb); double ra = lg.r * F_ / ca.z, rb = lg.r * F_ / cb.z;
    V2 dd = pb - pa; double l = std::hypot(dd.x, dd.y); if (l < 1) continue;
    V2 n{-dd.y / l, dd.x / l};
    uint32_t start = (uint32_t)pool_.size();
    for (int k = 0; k <= 8; k++) { double t = std::atan2(n.y, n.x) + PI * k / 8; pool_.push_back(pb + V2{std::cos(t) * rb, std::sin(t) * rb}); }
    for (int k = 0; k <= 8; k++) { double t = std::atan2(-n.y, -n.x) + PI * k / 8; pool_.push_back(pa + V2{std::cos(t) * ra, std::sin(t) * ra}); }
    Col lc = fogged(LOG, depth);
    prims_.push_back({(float)depth, bandOf(depth), 2, 0x80000000u, lc, start, 18});
    uint32_t s2 = (uint32_t)pool_.size();  // moss along the top
    V2 up = n.y < 0 ? n : n * -1.0;
    pool_.push_back(pa + up * (ra * 0.95)); pool_.push_back(pb + up * (rb * 0.95)); pool_.push_back(pb + up * (rb * 0.45)); pool_.push_back(pa + up * (ra * 0.45));
    prims_.push_back({(float)(depth - 0.001), bandOf(depth), 3, 0x80000001u, fogged(MOSS, depth), s2, 4});
  }
  for (const Tree& t : trees_) { if (t.s - s_ > AHEAD + 30) break; addTree(t); }
  for (const Fern& fn : ferns_) {
    if (fn.s - s_ > AHEAD) break;
    V3 b = at(fn.s, fn.d); double depth = toCam(b).z; if (depth < NEAR_Z) continue;
    Col col = fogged(FERN[fn.c], depth);
    if (depth > 12) {  // further off a fern is just a low clump
      V3 R = norm(V3{fwd_.z, 0, -fwd_.x});
      addWorld({b + R * (-fn.size), b + R * (-fn.size * 0.6) + V3{0, fn.size * 0.55, 0}, b + V3{0, fn.size * 0.7, 0},
                b + R * (fn.size * 0.6) + V3{0, fn.size * 0.5, 0}, b + R * fn.size}, depth, col, 2);
      continue;
    }
    for (int k = 0; k < 6; k++) {  // fronds arching out in every direction
      double a = fn.rot + k * 2 * PI / 6, len = fn.size * (0.75 + 0.45 * hash(fn.s + k));
      V3 dir{std::cos(a), 0, std::sin(a)}, side{-dir.z, 0, dir.x};
      V3 pts[8]; int n = 0;
      const double ts[4] = {0, 0.33, 0.66, 1};
      for (int i = 0; i < 4; i++) {
        double t = ts[i], w = len * 0.13 * std::sin(PI * std::min(t, 0.92));
        pts[n++] = b + dir * (len * t) + V3{0, len * (1.0 * t - 0.9 * t * t) + 0.02, 0} + side * w;
      }
      for (int i = 3; i >= 0; i--) {
        double t = ts[i], w = len * 0.13 * std::sin(PI * std::min(t, 0.92));
        pts[n++] = b + dir * (len * t) + V3{0, len * (1.0 * t - 0.9 * t * t) + 0.02, 0} - side * w;
      }
      V3 cam[8]; for (int i = 0; i < 8; i++) cam[i] = toCam(pts[i]);
      addPoly(cam, 8, depth - 0.001 * k, col, 2);
    }
  }
  // shafts of light through the canopy: behind the nearer trees, in front of the far ones
  V3 sun{std::sin(sunAz_) * std::cos(28 * DEG), std::sin(28 * DEG), std::cos(sunAz_) * std::cos(28 * DEG)};
  V3 sc = toCam(eye_ + sun * 1000);
  if (sc.z > 0 && sunFacing_ > 0.02) {
    V2 sp = proj(sc); V2 down = proj(toCam(eye_ + sun * 1000 - V3{0, 600, 0})) - sp;
    double base = std::atan2(down.y, down.x), L = H * 2.2;
    for (int i = 0; i < 7; i++) {
      double a = base + (hash(seed_ + i * 7.1) - 0.5) * 1.1 + 0.03 * std::sin(time_ * 0.17 + i);
      double wdt = 0.02 + 0.045 * hash(seed_ + i * 3.7), al = 0.1 * sunFacing_ * (0.45 + 0.55 * std::sin(time_ * 0.21 + i * 1.9) * std::sin(time_ * 0.13 + i));
      if (al <= 0.004) continue;
      uint32_t start = (uint32_t)pool_.size();
      pool_.push_back(sp); pool_.push_back(sp + V2{std::cos(a - wdt) * L, std::sin(a - wdt) * L}); pool_.push_back(sp + V2{std::cos(a + wdt) * L, std::sin(a + wdt) * L});
      prims_.push_back({(float)(14 + i * 0.01), bandOf(14), 9, (uint32_t)i, rgba(255, 246, 222, al), start, 3});
    }
  }
}

void Forest::drawPrims() {
  std::sort(prims_.begin(), prims_.end(), [](const Prim& a, const Prim& b) {
    if (a.band != b.band) return a.band > b.band;
    if (a.layer != b.layer) return a.layer < b.layer;
    if (a.key != b.key) return a.key < b.key;
    return a.depth > b.depth;
  });
  size_t i = 0;
  while (i < prims_.size()) {  // fill runs of the same colour as one path
    if (prims_[i].layer == 9) {  // a shaft of light: soft, fading away from the sun
      const Prim& p = prims_[i]; V2 apex = pool_[p.start], tail = (pool_[p.start + 1] + pool_[p.start + 2]) * 0.5;
      ctx.beginPath(); ctx.moveTo(apex); ctx.lineTo(pool_[p.start + 1]); ctx.lineTo(pool_[p.start + 2]); ctx.closePath();
      ctx.fillLinear(apex, tail, {{0, alpha(p.col, 0)}, {0.25, p.col}, {0.7, alpha(p.col, p.col.a * 0.5)}, {1, alpha(p.col, 0)}});
      i++; continue;
    }
    Col c = prims_[i].col; ctx.beginPath();
    size_t j = i;
    for (; j < prims_.size() && same(prims_[j].col, c); j++) {
      const Prim& p = prims_[j];
      ctx.moveTo(pool_[p.start]);
      for (uint32_t k = 1; k < p.count; k++) ctx.lineTo(pool_[p.start + k]);
      ctx.closePath();
    }
    ctx.fillStyle = c; ctx.fill();
    i = j;
  }
}

void Forest::drawMotes(double dt) {
  // dust and pollen hanging in the air, drifting past
  for (Mote& m : motes_) {
    m.p.z -= v_ * dt; m.p.x += 0.05 * std::sin(time_ * 0.4 + m.ph) * dt; m.p.y += 0.04 * std::sin(time_ * 0.31 + m.ph * 2) * dt;
    if (m.p.z < 0.4) m.p = {rnd() * 8 - 4, rnd() * 3 - 1.2, 7 + rnd() * 3};
    if (m.p.z < 0.6 || m.p.y < -0.9) continue;
    V2 p = {cx_ + F_ * m.p.x / m.p.z, cy_ - F_ * m.p.y / m.p.z};
    double r = std::max(0.5, F_ * 0.0012 * m.r / m.p.z), al = (0.12 + 0.4 * sunFacing_) * smooth(0.6, 1.5, m.p.z) * (1 - smooth(6, 9, m.p.z));
    ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, 2 * PI); ctx.fillStyle = rgba(250, 244, 226, al); ctx.fill();
  }
}

/* ---------- the cockpit ---------- */
// Bike space: x right, y up, z forward, origin at the rider's eye when the suspension is at rest.
namespace {
const V3 STEER_TOP{0, -0.53, 0.47};
const V3 STEER_AXIS{0, -std::sin(65 * DEG), std::cos(65 * DEG)};  // 65° head angle, pointing down the steerer
const V3 CLAMP{0, -0.495, 0.505};  // 40 mm stem with a little rise
}

V3 Forest::bikeToCam(V3 b) const {
  double a = LOOK_DOWN + pitch_ * 0.25, ca = std::cos(a), sa = std::sin(a);
  b.y += barY_ - eyeY_;  // the bike moves on its suspension a little more than the rider's head does
  return {b.x, b.y * ca + b.z * sa, b.z * ca - b.y * sa};
}
bool Forest::bikeProj(V3 b, V2& out, double& z) const {
  V3 c = bikeToCam(b); z = c.z; if (c.z < 0.05) return false;
  out = {cx_ + F_ * c.x / c.z, cy_ - F_ * c.y / c.z}; return true;
}

// a tube through bike-space points with a radius at each, drawn as one outline plus a highlight along its top
void Forest::tube(const std::vector<V3>& pts, const std::vector<double>& radii, Col col, Col hi, double hiAlpha) {
  std::vector<V2> c; std::vector<double> w;
  const double zn = 0.06;
  auto put = [&](V3 cam, double r) { c.push_back({cx_ + F_ * cam.x / cam.z, cy_ - F_ * cam.y / cam.z}); w.push_back(r * F_ / cam.z); };
  for (size_t i = 0; i < pts.size(); i++) {
    V3 a = bikeToCam(pts[i]);
    if (i > 0) {  // entering or leaving the space behind the near plane: cut the tube there
      V3 b = bikeToCam(pts[i - 1]);
      if ((a.z >= zn) != (b.z >= zn)) { double t = (zn - b.z) / (a.z - b.z); put(b + (a - b) * t, lerp(radii[i - 1], radii[i], t)); }
    }
    if (a.z >= zn) put(a, radii[i]);
  }
  if (c.size() < 2) return;
  std::vector<V2> L, R;
  for (size_t i = 0; i < c.size(); i++) {
    V2 d = (i + 1 < c.size() ? c[i + 1] : c[i]) - (i > 0 ? c[i - 1] : c[i]);
    double l = std::hypot(d.x, d.y); if (l < 1e-6) l = 1;
    V2 n{-d.y / l, d.x / l};
    L.push_back(c[i] + n * w[i]); R.push_back(c[i] - n * w[i]);
  }
  ctx.fillStyle = col;
  ctx.beginPath(); ctx.moveTo(L[0]);
  for (size_t i = 1; i < L.size(); i++) ctx.lineTo(L[i]);
  {  // round cap at the far end, then back along the other side, round cap at the start
    size_t e = c.size() - 1; double a0 = std::atan2(L[e].y - c[e].y, L[e].x - c[e].x);
    for (int k = 1; k < 8; k++) { double a = a0 - PI * k / 8; ctx.lineTo(c[e] + V2{std::cos(a) * w[e], std::sin(a) * w[e]}); }
  }
  for (size_t i = R.size(); i-- > 0;) ctx.lineTo(R[i]);
  {
    double a0 = std::atan2(R[0].y - c[0].y, R[0].x - c[0].x);
    for (int k = 1; k < 8; k++) { double a = a0 - PI * k / 8; ctx.lineTo(c[0] + V2{std::cos(a) * w[0], std::sin(a) * w[0]}); }
  }
  ctx.closePath(); ctx.fill();
  // highlight on whichever edge faces up the screen (the sky is the light)
  ctx.beginPath();
  for (size_t i = 0; i < c.size(); i++) {
    V2 edge = (L[i].y < R[i].y ? L[i] : R[i]); V2 p = c[i] + (edge - c[i]) * 0.55;
    if (i) ctx.lineTo(p); else ctx.moveTo(p);
  }
  double wm = 0; for (double x : w) wm += x; wm /= w.size();
  ctx.strokeStyle = alpha(hi, hiAlpha); ctx.lineWidth = std::max(0.6, wm * 0.35); ctx.roundCap = true; ctx.roundJoin = true; ctx.stroke();
}

void Forest::blob(const std::vector<V3>& pts, Col col) {
  ctx.beginPath(); bool first = true;
  for (const V3& b : pts) { V2 p; double z; if (!bikeProj(b, p, z)) continue; if (first) ctx.moveTo(p); else ctx.lineTo(p); first = false; }
  ctx.closePath(); ctx.fillStyle = col; ctx.fill();
}

void Forest::drawCockpit() {
  double comp = comp_, d = steer_;
  auto steer = [&](V3 p) { return STEER_TOP + rotAxis(p - STEER_TOP, STEER_AXIS, d); };
  V3 crown = STEER_TOP + STEER_AXIS * 0.17;
  V3 X = rotAxis({1, 0, 0}, STEER_AXIS, d);
  // fork: lowers and the axle ride up the stanchions as it compresses
  V3 axle = crown + STEER_AXIS * (0.55 - comp) + V3{0, 0, 0.04};
  axle = steer(axle);
  // front tyre: the top of it, between the fork legs, with the tread rolling away from the rider
  {
    V3 Wf = rotAxis({0, 0, 1}, STEER_AXIS, d), Wu{0, 1, 0};
    const double R = 0.37;
    std::vector<V3> band; std::vector<V2> scr;
    auto tp = [&](double t, double x, double rr) { return axle + Wu * (std::cos(t) * rr) + Wf * (std::sin(t) * rr) + X * x; };
    // only the stretch of tread facing the rider: further round, the outline would fold back over itself on screen
    for (int i = 0; i <= 16; i++) { double t = (-45 + 85 * i / 16.0) * DEG; band.push_back(tp(t, -0.031, R - 0.01)); }
    for (int i = 16; i >= 0; i--) { double t = (-45 + 85 * i / 16.0) * DEG; band.push_back(tp(t, 0.031, R - 0.01)); }
    blob(band, TIRE);
    ctx.beginPath();  // knobs, all in one fill
    for (int k = 0; k < 40; k++) {
      double t = std::fmod(k * 9 * DEG + wheel_, 2 * PI); if (t > PI) t -= 2 * PI;
      if (t < -42 * DEG || t > 24 * DEG) continue;
      for (double x : {-0.024, -0.009, 0.009, 0.024}) {
        if ((k % 2) && std::fabs(x) < 0.015) continue;
        double dt = 0.012; bool ok = true; V2 q[4]; double z;
        const V3 corners[4] = {tp(t - dt, x - 0.006, R + 0.006), tp(t - dt, x + 0.006, R + 0.006), tp(t + dt, x + 0.006, R + 0.006), tp(t + dt, x - 0.006, R + 0.006)};
        for (int c = 0; c < 4; c++) ok = ok && bikeProj(corners[c], q[c], z);
        if (!ok) continue;
        ctx.moveTo(q[0]); ctx.lineTo(q[1]); ctx.lineTo(q[2]); ctx.lineTo(q[3]); ctx.closePath();
      }
    }
    ctx.fillStyle = KNOB; ctx.fill();
  }
  // fork legs: black lowers, gold stanchions, the crown
  double stanVis = 0.205 - comp;
  for (int side = -1; side <= 1; side += 2) {
    V3 legTop = steer(crown + V3{side * 0.058, 0, 0}), legSeal = steer(crown + V3{side * 0.058, 0, 0} + STEER_AXIS * stanVis);
    V3 legAxle = steer(crown + V3{side * 0.058, 0, 0} + STEER_AXIS * (0.55 - comp) + V3{0, 0, 0.04});
    tube({legSeal, legAxle}, {0.024, 0.022}, BLACK, RIM, 0.35);
    tube({legTop, legSeal}, {0.0175, 0.0175}, GOLD, GOLD_HI, 0.8);
  }
  tube({steer(crown + V3{-0.075, 0.005, -0.01}), steer(crown + V3{0.075, 0.005, -0.01})}, {0.026, 0.026}, BLACK, RIM, 0.45);
  // frame: head tube, and the top and down tubes running back under the rider
  V3 htTop = STEER_TOP + STEER_AXIS * 0.012, htBot = STEER_TOP + STEER_AXIS * 0.15;
  tube({htBot + V3{0, -0.01, 0}, V3{0, -1.25, 0.18}}, {0.034, 0.04}, FRAME, FRAME_HI, 0.45);
  tube({STEER_TOP + STEER_AXIS * 0.04, V3{0, -0.82, 0.08}}, {0.028, 0.03}, FRAME, FRAME_HI, 0.5);
  tube({htTop, htBot}, {0.031, 0.034}, FRAME, FRAME_HI, 0.5);
  // stem and top cap
  tube({steer(STEER_TOP + V3{0, 0.005, -0.005}), steer(CLAMP)}, {0.021, 0.021}, BLACK, RIM, 0.55);
  {
    V2 p; double z;
    if (bikeProj(steer(STEER_TOP + V3{0, 0.03, 0}), p, z)) { ctx.beginPath(); ctx.arc(p.x, p.y, 0.017 * F_ / z, 0, 2 * PI); ctx.fillStyle = DARK; ctx.fill(); ctx.strokeStyle = alpha(RIM, 0.5); ctx.lineWidth = 1; ctx.stroke(); }
  }
  // handlebar: 780 mm, 25 mm rise, 9° backsweep
  auto barPt = [&](double x) {
    double ax = std::fabs(x), rise = 0.025 * smooth(0.07, 0.17, ax), back = std::max(0.0, ax - 0.08) * 0.155;
    return steer(CLAMP + V3{x, rise, -back});
  };
  std::vector<V3> bar; std::vector<double> br;
  for (int i = 0; i <= 26; i++) { double x = -0.39 + 0.78 * i / 26; bar.push_back(barPt(x)); br.push_back(std::fabs(x) < 0.05 ? 0.0165 : std::fabs(x) < 0.12 ? lerp(0.0165, 0.0115, (std::fabs(x) - 0.05) / 0.07) : 0.0115); }
  tube(bar, br, BLACK, RIM, 0.6);
  // stem clamp face over the bar
  tube({steer(CLAMP + V3{-0.026, 0.002, 0.008}), steer(CLAMP + V3{0.026, 0.002, 0.008})}, {0.02, 0.02}, DARK, RIM, 0.5);
  for (int side = -1; side <= 1; side += 2) {
    V3 G = barPt(side * 0.325);
    auto hand = [&](double u, double v, double w) { return G + rotAxis({side * u, v, w}, STEER_AXIS, d); };
    // brake: master cylinder on the bar and a lever blade running out under the index finger
    tube({barPt(side * 0.2), barPt(side * 0.245)}, {0.014, 0.014}, DARK, RIM, 0.4);
    tube({barPt(side * 0.24) + rotAxis({0, -0.006, 0.02}, STEER_AXIS, d), hand(-0.06, -0.004, 0.05)}, {0.006, 0.005}, DARK, RIM, 0.5);
    // grip
    tube({barPt(side * 0.262), barPt(side * 0.39)}, {0.0175, 0.0175}, rgba(30, 30, 30), rgba(80, 80, 80), 0.3);
    // forearm and sleeve, coming in from below
    tube({hand(0.0, 0.012, -0.085), hand(0.07, -0.07, -0.4)}, {0.032, 0.05}, SLEEVE, SLEEVE_HI, 0.35);
    // thumb under the bar, back of the hand over the grip, fingers wrapped round its front, index finger on the lever
    tube({hand(-0.044, 0.006, -0.05), hand(-0.055, -0.004, -0.02)}, {0.012, 0.011}, GLOVE, GLOVE_HI, 0.15);
    blob({hand(-0.045, 0.018, -0.078), hand(-0.05, 0.03, -0.03), hand(-0.044, 0.036, 0.004), hand(0.0, 0.04, 0.01),
          hand(0.044, 0.035, 0.006), hand(0.05, 0.027, -0.03), hand(0.041, 0.017, -0.078)}, GLOVE);
    for (double u : {-0.013, 0.01, 0.031}) {
      double rr = u > 0.02 ? 0.0085 : 0.0095;
      tube({hand(u, 0.035, 0.004), hand(u, 0.031, 0.022)}, {rr, rr}, GLOVE, GLOVE_HI, 0.3);
    }
    tube({hand(-0.035, 0.033, 0.006), hand(-0.066, 0.01, 0.044)}, {0.0095, 0.0085}, GLOVE, GLOVE_HI, 0.35);
    tube({hand(-0.042, 0.034, -0.01), hand(0.0, 0.039, -0.004), hand(0.043, 0.033, -0.008)}, {0.003, 0.003, 0.003}, alpha(GLOVE_HI, 0.5), GLOVE_HI, 0.0);
    // glove cuff
    tube({hand(-0.042, 0.018, -0.08), hand(0.0, 0.026, -0.083), hand(0.04, 0.018, -0.08)}, {0.006, 0.006, 0.006}, CUFF, CUFF, 0.0);
  }
}

/* ---------- loop ---------- */
void Forest::frame(double dtIn) {
  double dt = std::min(0.05, dtIn);
  time_ += dt;
  // speed: faster down the slopes, easing back toward a relaxed pace
  v_ = clampd(v_ + (-9.81 * slope(s_ + 2) * 0.55 + (5.2 - v_) * 0.45) * dt, 3.2, 9.5);
  s_ += v_ * dt;
  extendTrail(s_ + AHEAD + 80);
  generate(s_ + AHEAD + 30);
  // suspension: the front wheel (0.8 m ahead) and rear (0.45 m behind) ride over roots and rocks; the bars
  // follow through the fork, and the rider's head follows more softly still
  double bf = bumpAt(s_ + 0.8), brr = bumpAt(s_ - 0.45);
  const double wf = 2 * PI * 2.4, we = 2 * PI * 1.3;
  for (int i = 0; i < 4; i++) {
    double h = dt / 4;
    barV_ += (wf * wf * (bf - barY_) - 2 * 0.45 * wf * barV_) * h; barY_ += barV_ * h;
    rearY_ += (brr - rearY_) * std::min(1.0, h * 25);
    eyeV_ += (we * we * (barY_ * 0.6 + rearY_ * 0.4 - eyeY_) - 2 * 0.6 * we * eyeV_) * h; eyeY_ += eyeV_ * h;
  }
  comp_ = clampd(0.045 + (bf - barY_) * 1.4 + std::max(0.0, -barV_) * 0.01, 0.0, 0.16);
  wheel_ += v_ * dt / 0.37;
  // line choice: drift a little across the trail
  latV_ += (-(lat_ - 0.12 * std::sin(s_ / 6.3 + ph_[10])) * 6 - latV_ * 3) * dt; lat_ += latV_ * dt;
  // look where the trail goes, lean into the turns, pitch with the grade
  double look = 0; for (int k = 0; k < 5; k++) look += theta(s_ + 2.5 + k * 1.5); look /= 5;
  yaw_ += (look - yaw_) * std::min(1.0, dt * 3);
  double kappa = curvature(s_ + 2);
  roll_ += (-std::atan(v_ * v_ * kappa / 9.81) * 0.75 - roll_) * std::min(1.0, dt * 2.5);
  double gp = std::atan(slope(s_ + 3)) * 0.7 + std::atan((bf - brr) / 1.25) * 0.4;
  pitch_ += (gp - pitch_) * std::min(1.0, dt * 4);
  steer_ += (clampd(std::atan(kappa * 1.2) * 0.55 + latV_ * 0.05, -0.12, 0.12) - steer_) * std::min(1.0, dt * 5);
  // camera
  V3 p = at(s_, lat_); eye_ = {p.x, height(s_) + EYE + eyeY_, p.z};
  double ph = pitch_ - LOOK_DOWN - (barY_ - eyeY_) * 0.3, cy = std::cos(yaw_), sy = std::sin(yaw_), cp = std::cos(ph), sp = std::sin(ph);
  V3 f{sy * cp, sp, cy * cp}, r0{cy, 0, -sy}, u0{-sy * sp, cp, -cy * sp};
  double cr = std::cos(roll_), sr = std::sin(roll_);
  fwd_ = f; right_ = r0 * cr + u0 * sr; up_ = u0 * cr - r0 * sr;
  double facing = std::cos(sunAz_ - yaw_); sunFacing_ = facing > 0 ? facing * facing : 0;
  fog_ = mix(FOG, FOG_SUN, sunFacing_ * 0.55);

  pool_.clear(); prims_.clear();
  ctx.begin();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawSky();
  drawGround();
  buildWorld();
  drawPrims();
  drawMotes(dt);
  drawCockpit();
  // a soft vignette, like a lens
  ctx.beginPath(); ctx.rect(-4, -4, W + 8, H + 8);
  ctx.fillRadial({cx_, cy_ * 0.9}, W * 0.75, H * 0.85, {{0, rgba(0, 0, 0, 0)}, {0.6, rgba(0, 0, 0, 0)}, {1, rgba(8, 10, 10, 0.5)}});
  ctx.end();
}
