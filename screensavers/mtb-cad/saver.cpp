#define _WIN32_WINNT 0x0A00  // Windows 10: GetDpiForWindow
// Port of the MTB CAD page (web/index.html) to native drawing. The structure, names and comments follow the
// page so the two can be compared side by side; units are millimetres and seconds, bike space is y-up with
// the rear axle at the origin.
#include "saver.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
const double PI = 3.141592653589793;
double lerp(double a, double b, double t) { return a + (b - a) * t; }
V2 lerp2(V2 a, V2 b, double t) { return {lerp(a.x, b.x, t), lerp(a.y, b.y, t)}; }
V2 along(V2 a, V2 dir, double d) { return {a.x + dir.x * d, a.y + dir.y * d}; }
struct U3 { double x, y, l; };
U3 unit(V2 a, V2 b) { double dx = b.x - a.x, dy = b.y - a.y, l = std::hypot(dx, dy); return {dx / l, dy / l, l}; }
double clampd(double v, double a, double b) { return std::max(a, std::min(b, v)); }
double smooth(double e0, double e1, double x) { double t = clampd((x - e0) / (e1 - e0), 0, 1); return t * t * (3 - 2 * t); }
V2 rotAbout(V2 p, V2 c, double a) { double dx = p.x - c.x, dy = p.y - c.y; return {c.x + dx * std::cos(a) - dy * std::sin(a), c.y + dx * std::sin(a) + dy * std::cos(a)}; }
double jsRound(double x) { return std::floor(x + 0.5); }

/* ---------- CAD layers ---------- */
const Col GROUND = rgba(13, 19, 22), LINE = rgba(210, 218, 221), THIN = rgba(210, 218, 221, 0.5), DIM = rgba(127, 163, 176),
          CENTER = rgba(163, 148, 120), GRID = rgba(127, 163, 176, 0.06), BORDER = rgba(210, 218, 221, 0.6);
Col dimA(double a) { return rgba(127, 163, 176, a); }
Col centerA(double a) { return rgba(163, 148, 120, a); }
Col lineA(double a) { return rgba(210, 218, 221, a); }
const double DASH_CENTER[4] = {16, 3, 3, 3};

/* ---------- bike geometry (mm). Rear axle at origin. 29er, 65° head angle, 44 mm fork offset ---------- */
const double HA = 65 * PI / 180;
const V2 HDIR{std::cos(HA), -std::sin(HA)}, HPERP{std::sin(HA), std::cos(HA)};
const V2 REAR{0, 0}, BB{440, -30};
// 600 mm axle-to-crown so the tyre clears the crown and down tube at full travel; the head tube sits on the
// same steering axis, raised along it, so the wheelbase and head angle don't change
const double FORK_EXT = 120;
V2 FX(V2 p) { return {p.x - HDIR.x * FORK_EXT, p.y - HDIR.y * FORK_EXT}; }
const V2 HT_TOP = FX({985, 560}), HT_BOT = along(HT_TOP, HDIR, 150), CROWN = along(HT_TOP, HDIR, 160);
const V2 FRONT = [] { double t = (CROWN.y + 44 * HPERP.y) / -HDIR.y; V2 p = along(along(CROWN, HDIR, t), HPERP, 44); return V2{jsRound(p.x), 0}; }();
const V2 ST{325, 480}, TT_END = along(HT_TOP, HDIR, 40), DT_END = along(HT_TOP, HDIR, 118);
const V2 PIVOT{470, 40}, ROCK[3] = {{360, 300}, {400, 262}, {478, 326}};
const V2 SHOCK_A{640, 135}, SHOCK_B = ROCK[2];
const V2 POST_DIR = [] { U3 u = unit(BB, ST); return V2{u.x, u.y}; }();
const double SEAT_ANGLE = std::atan2(ST.y - BB.y, BB.x - ST.x) * 180 / PI;
const double SHOCK_L0 = unit(SHOCK_A, ROCK[2]).l;
const V2 FORK_DIR = [] { U3 u = unit(CROWN, FRONT); return V2{u.x, u.y}; }();
const V2 SADDLE = along(ST, POST_DIR, 185);

/* ---------- ride simulation (mm, s): sprung frame with heave + pitch on two spring-dampers ---------- */
const V2 COMB{647, 0};
const double GRAV = 9810, V = 3000, SLOWMO = 0.55, R_TIRE = 385;
const double TRAVEL_F = 160, TRAVEL_R = 150, SAG_F = 48, SAG_R = 45;
const double KF = GRAV / (2 * SAG_F), KR = GRAV / (2 * SAG_R), RG2 = 260000;
const double GRADE = 0.07, LEAD = 3000, FACE = 1000, BX_OFFSET = -250, BOULDER_X = 2050;
const V2 BOULDER[13] = {{0, 0}, {60, 150}, {120, 250}, {175, 270}, {250, 400}, {340, 455}, {430, 470}, {520, 440}, {600, 455}, {690, 360}, {780, 230}, {860, 120}, {930, 0}};
const double FALL_ZOOM = 0.6;  // camera pulls back this far for the drop
const V2 BIKE_C{645, 170};
const double DUR_CRASH = 4.0, DUR_REASSEMBLE = 2.6, RIDE_TIMEOUT = 20;
const int COGS[12] = {10, 12, 14, 16, 18, 21, 24, 28, 32, 36, 42, 52};
enum { RIDE, CRASH, REASSEMBLE };
enum { FRAME, SWINGARM, SHOCK, FORK, FRONTWHEEL, REARWHEEL, ROTORF, ROTORR, CASSETTE, CRANKSET, DERAILLEUR, COCKPIT, DROPPER };
const int ORDER[13] = {ROTORR, REARWHEEL, CASSETTE, SWINGARM, SHOCK, FRAME, DROPPER, FORK, ROTORF, FRONTWHEEL, COCKPIT, DERAILLEUR, CRANKSET};

double hs2(double n) { double s = std::sin(n * 127.1 + 917.3) * 43758.5453; return s - std::floor(s); }  // fixed seed (soil stones)
double ease(double t) { return t < 0.5 ? 4 * t * t * t : 1 - std::pow(-2 * t + 2, 3) / 2; }

std::vector<V2> ring12(double r) { std::vector<V2> o; for (int i = 0; i < 12; i++) o.push_back({std::cos(i * PI / 6) * r, std::sin(i * PI / 6) * r}); return o; }
// collision outline of each part: points in bike space (or around the axle for wheel-centred parts)
std::vector<V2> hullFor(int id) {
  switch (id) {
    case FRONTWHEEL: case REARWHEEL: return ring12(390);
    case ROTORF: return ring12(102);
    case ROTORR: return ring12(91);
    case CASSETTE: return ring12(112);
    case CRANKSET: { auto h = ring12(72); h.push_back({170, 0}); h.push_back({-170, 0}); return h; }
    case FRAME: return {BB, ST, TT_END, HT_TOP, HT_BOT, DT_END, {ST.x + 120, ST.y - 20}, lerp2(BB, DT_END, 0.5)};
    case SWINGARM: return {PIVOT, REAR, ROCK[0], ROCK[1], ROCK[2], lerp2(PIVOT, REAR, 0.5)};
    case SHOCK: return {SHOCK_A, SHOCK_B, lerp2(SHOCK_A, SHOCK_B, 0.5)};
    case FORK: return {CROWN, HT_TOP, FRONT, lerp2(CROWN, FRONT, 0.5)};
    case COCKPIT: return {HT_TOP, FX({1040, 585}), FX({1002, 612}), FX({1102, 566})};
    case DROPPER: return {ST, SADDLE, {SADDLE.x - 150, SADDLE.y + 30}, {SADDLE.x + 160, SADDLE.y + 14}};
    default: return {{18, -12}, {62, -36}, {15, -75}, {40, -145}, {62, -50}};  // derailleur
  }
}
// where each part gets labelled on the assembled bike: anchor point in bike space, leader angle (deg), leader length (px)
struct Tag { V2 anchor; double deg, len; };
Tag tagFor(int id) {
  switch (id) {
    case FRAME: return {lerp2(ST, TT_END, 0.6), 75, 70};
    case SWINGARM: return {{150, 125}, 165, 70};
    case SHOCK: return {{560, 225}, 35, 95};
    case FORK: return {lerp2(CROWN, FRONT, 0.72), 5, 60};
    case FRONTWHEEL: return {{1295 + 270, 270}, 40, 50};
    case REARWHEEL: return {{-270, 270}, 140, 50};
    case ROTORF: return {{1295 - 60, -75}, -115, 60};
    case ROTORR: return {{-55, -70}, -115, 60};
    case CASSETTE: return {{70, 75}, 100, 150};
    case CRANKSET: return {{440 + 60, -95}, -70, 55};
    case DERAILLEUR: return {{40, -160}, -95, 45};
    case COCKPIT: return {FX({1050, 600}), 60, 45};
    default: return {{230, 690}, 120, 50};  // dropper
  }
}
std::string upper(std::string s) { for (char& c : s) if (c >= 'a' && c <= 'z') c -= 32; return s; }
}  // namespace

/* ---------- setup ---------- */
Scene* CreateScene() { return new Saver; }
const wchar_t* const kSaverName = L"MTB CAD";
const wchar_t* const kSaverAbout = L"A CAD drawing of a mountain bike that rides, crashes and rebuilds itself.";

bool Saver::init(ID2D1Factory* f, IDWriteFactory* dw, HWND hwnd, unsigned seed) {
  hwnd_ = hwnd; rng_.seed(seed);
  if (!ctx.init(f, dw, hwnd)) return false;
  struct Def { int id; V2 home; int hp; double r; const char* name; const char* spec; bool local; };
  const Def defs[13] = {
      {FRAME, {690, 260}, 0, 430, "Front triangle", "HA 65° · SA 77°", false},
      {SWINGARM, {250, 150}, 0, 260, "Rear linkage", "150 mm travel", false},
      {SHOCK, lerp2(SHOCK_A, SHOCK_B, 0.5), 0, 130, "Coil shock", "205 × 65 mm", false},
      {FORK, lerp2(CROWN, FRONT, 0.45), 0, 260, "Fork", "160 mm · 44 off", false},
      {FRONTWHEEL, FRONT, 1, 392, "Front wheel", "29 × 2.5 in, 32H", true},
      {REARWHEEL, REAR, 2, 392, "Rear wheel", "29 × 2.4 in, 32H", true},
      {ROTORF, FRONT, 1, 104, "Front rotor", "Ø203 · 6-bolt", true},
      {ROTORR, REAR, 2, 92, "Rear rotor", "Ø180 · 6-bolt", true},
      {CASSETTE, REAR, 2, 115, "Cassette", "10–52T 12-spd", true},
      {CRANKSET, BB, 0, 200, "Crankset", "32T NW · 170", true},
      {DERAILLEUR, {30, -85}, 0, 90, "Derailleur", "Clutch 12-spd", false},
      {COCKPIT, FX({1040, 585}), 0, 90, "Cockpit", "800 bar · 40 stem", false},
      {DROPPER, {290, 600}, 0, 140, "Dropper post", "170 mm drop", false},
  };
  for (int i = 0; i < 13; i++) { const Def& d = defs[i]; Part p{}; p.id = d.id; p.home = d.home; p.hp = d.hp; p.r = d.r; p.name = d.name; p.spec = d.spec; p.local = d.local; p.item = i + 1; parts.push_back(p); }
  rideReset(0);
  doResize();
  setState(RIDE);
  return true;
}

void Saver::resized() { RECT rc; GetClientRect(hwnd_, &rc); ctx.resize(rc.right, rc.bottom); doResize(); }

void Saver::doResize() {
  RECT rc; GetClientRect(hwnd_, &rc);
  dpr = GetDpiForWindow(hwnd_) / 96.0; if (dpr <= 0) dpr = 1;
  W = rc.right / dpr; H = rc.bottom / dpr;
  wide = W >= 900 && W / H > 1.15; tiny = W < 400;
  mo = W < 600 ? 8 : 12; mi = W < 600 ? 22 : 30;
  if (tiny) { mo = 4; mi = 10; }
  double fx0 = mi, fy0 = mi, fx1 = W - mi, fy1 = H - mi;
  if (tiny) {  // no tables or labels: just the drawing
    panel = {fx1, fx1, fy1, fy1};
    A.x = fx0 + 3; A.y = fy0 + 3; A.w = fx1 - fx0 - 6; A.h = fy1 - fy0 - 6;
  } else if (wide) {
    double T = std::max(300.0, std::min(360.0, W * 0.21));
    panel = {fx1 - T, fx1, fy0, fy1};
    A.x = fx0 + 10; A.y = fy0 + 10; A.w = panel.x0 - fx0 - 20; A.h = fy1 - fy0 - 20;
  } else {
    panel = {fx0, fx1, fy1 - 62, fy1};
    A.x = fx0 + 6; A.y = fy0 + 6; A.w = fx1 - fx0 - 12; A.h = panel.y0 - fy0 - 12;
  }
  A.cx = A.x + A.w / 2; A.cy = A.y + A.h / 2;
  S0 = std::min(A.w * 0.88 / 2120, A.h * 0.88 / 1400); S = S0 * zoom;
}

/* ---------- terrain ---------- */
double Saver::hsh(double n) const { double s = std::sin(n * 127.1 + trailSeed * 311.7) * 43758.5453; return s - std::floor(s); }
// piecewise-linear value noise: straight facets between random heights, so the profile reads as broken ground
double Saver::lin(double u, double cell, double k) const {
  double i = std::floor(u / cell), f = u / cell - i;
  return lerp(hsh(i * 7.13 + k), hsh((i + 1) * 7.13 + k), f) * 2 - 1;
}
double Saver::rough(double u) const { return 105 * lin(u, 2100, 1) + 55 * lin(u, 700, 2) + 27 * lin(u, 230, 3) + 12 * lin(u, 85, 4) + 5 * lin(u, 31, 5); }
double Saver::rockH(double u, const Seg* T) {  // angular rocks: steep face, broken top, back slope
  if (!T || u <= 0 || u >= T->len) return 0;
  trailSeed = T->seed;
  double env = smooth(0, 1500, u), cell = 850, k = std::floor(u / cell), h = 0;
  for (double j = k - 1; j <= k + 1; j++) {
    if (j < 2 || hsh(j + 0.11) < 0.45 || j * cell > T->len - 2300) continue;
    double w1 = 90 + hsh(j + 0.3) * 130, wt = 70 + hsh(j + 0.5) * 260, w2 = 90 + hsh(j + 0.7) * 220;
    double s0 = j * cell + hsh(j + 0.9) * cell * 0.4, hr = (hsh(j + 1.3) < 0.3 ? 35 + hsh(j + 1.7) * 30 : 75 + hsh(j + 1.1) * 95) * env;
    double d = u - s0;
    if (d > 0 && d < w1 + wt + w2) {
      double top = hr * (1 + 0.18 * lin(d, 45, j * 3.1));
      h += d < w1 ? top * (d / w1) : d < w1 + wt ? top * (1 - 0.15 * (d - w1) / wt) : top * 0.85 * (1 - (d - w1 - wt) / w2);
    }
  }
  return h;
}
double Saver::trailH(double u, const Seg* T) {
  trailSeed = T->seed;
  double env = smooth(0, 1500, u), calm = 1 - 0.65 * smooth(T->len - 2600, T->len - 1700, u);
  double h = -GRADE * std::max(0.0, u - 1200) + env * calm * rough(u) + rockH(u, T);
  h -= 0.22 * clampd(u - (T->len - 7000), 0, 4200);                // steep chute before the run-in to the kicker
  h += 230 * clampd((u - (T->len - 1500)) / 1450, 0, 1);          // straight slab kicker before the drop
  return h;
}
// The trail is a chain of segments; each starts flat at the height where the last one ended:
// flat lead-in, rough descending trail, kicker, craggy cliff, boulder, then a flat run-out.
Saver::Seg* Saver::segAt(double x) { for (int i = (int)ride.segs.size() - 1; i >= 0; i--) if (x >= ride.segs[i].s) return &ride.segs[i]; return nullptr; }
double Saver::groundH(double x) {
  Seg* T = segAt(x); if (!T) return ride.segs.empty() ? 0 : ride.segs[0].y0;
  trailSeed = T->seed;
  double u = x - T->start; if (u <= 0) return T->y0;
  if (u < T->len) return T->y0 + trailH(u, T);
  double top = trailH(T->len, T), bottom = top - T->drop, v = u - T->len;
  if (v < FACE) {  // craggy cliff face: broken steps and spikes on the way down
    double t = v / FACE, crag = (110 * lin(v, 75, 9) + 50 * lin(v, 29, 10) + 20 * lin(v, 12, 11)) * std::sin(PI * std::min(1.0, t * 1.15));
    return T->y0 + lerp(top, bottom, std::pow(t, 0.6)) + crag;
  }
  return T->y0 + belowCliff(u, v, bottom, T) + boulderTop(v, T);
}
double Saver::belowCliff(double u, double v, double bottom, const Seg* T) {
  double R = runout(T), k = 1 - smooth(R, R + 1200, v);
  return bottom - GRADE * (std::min(v, R + 600) - FACE) + 0.7 * rough(u) * k;
}
double Saver::runout(const Seg* T) const { return (T->bx ? T->bx : BOULDER_X) + 930 + 400; }  // ground flattens out past the boulder
double Saver::boulderTop(double v, const Seg* T) const {
  double d = v - (T->bx ? T->bx : BOULDER_X); if (d <= 0 || d >= 930) return 0;
  int i = 0; while (BOULDER[i + 1].x < d) i++;
  return lerp(BOULDER[i].y, BOULDER[i + 1].y, (d - BOULDER[i].x) / (BOULDER[i + 1].x - BOULDER[i].x));
}
double Saver::contactY(double x) {  // height of the wheel centre resting on the ground under x
  double m = -1e9;
  for (int i = -12; i <= 12; i++) { double dx = i / 12.0 * R_TIRE * 0.92, y = groundH(x + dx) + std::sqrt(R_TIRE * R_TIRE - dx * dx); if (y > m) m = y; }
  return m;
}

/* ---------- ride ---------- */
void Saver::rideReset(double baseY) {
  ride.v = V; ride.z = baseY + R_TIRE - (SAG_F + SAG_R) / 2; ride.vz = 0; ride.th = 0; ride.vth = 0;
  ride.sf = SAG_F; ride.sr = SAG_R; ride.camZ = ride.z; ride.cf = ride.cr = baseY + R_TIRE;
}
void Saver::rideStep(double h) {
  if (!(h > 0)) return;
  double c = std::cos(ride.th), s = std::sin(ride.th);
  V2 fa{ride.X + 648 * c, ride.z + 648 * s}, ra{ride.X - 647 * c, ride.z - 647 * s};
  // tire give: the contact the suspension feels follows the ground with a short lag (rises softly, drops freely)
  double tf = contactY(fa.x), tr = contactY(ra.x), tc = std::min(1.0, h / 0.03);
  ride.cf = tf < ride.cf ? tf : ride.cf + (tf - ride.cf) * tc;
  ride.cr = tr < ride.cr ? tr : ride.cr + (tr - ride.cr) * tc;
  double sf = ride.cf - fa.y, sr = ride.cr - ra.y;
  auto force = [&](double sNow, double sPrev, double k, double trav) {
    if (sNow <= 0) return 0.0;
    double ds = sPrev > 0 ? (sNow - sPrev) / h : 0;
    // light compression damping, firmer rebound, and a blow-off cap so sharp edges don't kick the frame
    double damp = clampd((ds > 0 ? 0.7 : 2.6) * std::sqrt(k) * ds, -0.7 * GRAV, 0.7 * GRAV);
    return std::max(0.0, k * sNow + damp + (sNow > trav ? 40 * k * (sNow - trav) : 0));
  };
  double Ff = force(sf, ride.sf, KF, TRAVEL_F), Fr = force(sr, ride.sr, KR, TRAVEL_R);
  ride.vz += (Ff + Fr - GRAV) * h;
  if (Ff > 0 && Fr > 0) ride.vz *= 1 - 2.5 * h;  // rider's legs soak up heave while both wheels are down
  ride.z += ride.vz * h;
  ride.vth += ((Ff * 648 - Fr * 647) * c / RG2) * h; ride.vth *= 1 - (Ff > 0 && Fr > 0 ? 6 : 1.5) * h;
  ride.th = clampd(ride.th + ride.vth * h, -0.45, 0.45);
  // keep both tires on the ground while riding: if the ground falls away faster than the fork or shock can
  // extend, the rider weights that end and the frame follows it down. Only past the kicker lip can a wheel leave.
  double lip = ride.trail ? ride.trail->start + ride.trail->len - 30 : 1e300, L = 1295;
  // A wheel that has just rolled off the crest of a rock may float briefly (up to 60 mm) before it touches down.
  auto offRock = [&](double x) {
    Seg* T = ride.trail; if (!T) return 0.0;
    double u = x - T->start, g = rockH(u, T), top = g;
    for (int k = 1; k <= 8; k++) top = std::max(top, rockH(u - k * 50, T));
    return top - g > 110 ? std::min(60.0, (top - g) * 0.45) : 0.0;
  };
  double gf = sf, gr = sr;
  if (fa.x < lip && gf < 0) {
    double allow = offRock(fa.x);
    if (gf < -allow) {
      double d = -gf - allow; ride.z -= d * 647 / L; ride.th -= d / L; gf = -allow;
      double v = ride.vz + 648 * ride.vth; if (v > 0) { ride.vz -= v * 647 / L; ride.vth -= v / L; }
    }
  }
  if (ra.x < lip && gr < 0) {
    double allow = offRock(ra.x);
    if (gr < -allow) {
      double d = -gr - allow; ride.z -= d * 648 / L; ride.th += d / L; gr = -allow;
      double v = ride.vz - 647 * ride.vth; if (v > 0) { ride.vz -= v * 648 / L; ride.vth += v / L; }
    }
  }
  // speed: gravity along the slope accelerates the bike; on gentler ground the rider settles back toward cruising speed
  if (Ff > 0 || Fr > 0) {
    double slope = (groundH(ride.X + 600) - groundH(ride.X - 600)) / 1200;
    ride.v = clampd(ride.v + (-GRAV * slope * 0.9 + (V - ride.v) * 0.35) * h, 1500, 9000);
  }
  ride.sf = gf; ride.sr = gr; ride.X += ride.v * h;
}
// run the ride physics ahead on a copy of the state (no boulder) to find where the front wheel lands
double Saver::predictTouchdown(double cliff, Seg* T) {
  Ride saved = ride;
  T->bx = 1e7; double at = 2600;
  for (int i = 0; i < 1500; i++) {
    rideStep(0.001);
    double fx = ride.X + 648 * std::cos(ride.th);
    if (fx > cliff + 520 && ride.sf > 0) { at = fx - cliff; break; }
  }
  ride.X = saved.X; ride.v = saved.v; ride.z = saved.z; ride.vz = saved.vz; ride.th = saved.th; ride.vth = saved.vth;
  ride.sf = saved.sf; ride.sr = saved.sr; ride.cf = saved.cf; ride.cr = saved.cr;
  return at;
}
Saver::Pose Saver::pose() const {
  double tf = clampd(ride.sf, 0, TRAVEL_F), tr = clampd(ride.sr, 0, TRAVEL_R), phi = -tr / 470, psi = 2 * phi;
  return {tf, tr, phi, along(FRONT, FORK_DIR, -tf), rotAbout(REAR, PIVOT, phi), rotAbout(ROCK[0], ROCK[1], psi), rotAbout(ROCK[2], ROCK[1], psi)};
}

/* ---------- timeline ---------- */
// ride: labels, trail, kicker, cliff → front wheel hits the boulder → crash: the bike comes apart against it and the parts fall and settle
// → reassemble: parts lift off the ground and fly back together on the run-out → next ride
double Saver::assembly() const { return state == RIDE ? 1 : state == REASSEMBLE ? ease(std::min(1.0, st / DUR_REASSEMBLE)) : 0; }
V2 Saver::partPivot(const Part& p) const { return p.hp == 1 ? P.front : p.hp == 2 ? P.rear : p.home; }

void Saver::setState(int s) {
  state = s; st = 0;
  if (s == RIDE) {  // lay the next stretch of trail ahead, starting flat where the bike stands
    double L = V * SLOWMO, fs = ride.X + L * 4.9 + 648;
    Seg seg{fs - LEAD, fs, L * 5.8, 2600, rnd() * 100};
    seg.y0 = groundH(seg.s);
    ride.segs.push_back(seg); if (ride.segs.size() > 3) ride.segs.pop_front();
    ride.trail = &ride.segs.back();
    rideReset(groundH(ride.X));
  }
  if (s == CRASH) {  // the front wheel stops dead against the boulder and the bike comes apart around it
    V2 c0 = haveCrash ? crash.w : V2{ride.X + 648, ride.z};
    debris = burst(c0.x, c0.y + 40, 34, true);
    auto more = burst(c0.x - 150, c0.y + 250, 16, false); debris.insert(debris.end(), more.begin(), more.end());
    crashZoom = zoom;
    for (Part& p : parts) {
      V2 piv = partPivot(p), w = bikeWorld(piv);
      std::vector<V2> hull = hullFor(p.id);
      if (!p.local) for (V2& q : hull) q = q - piv;
      double rr = 60; for (V2 q : hull) rr = std::max(rr, std::hypot(q.x, q.y));
      // velocity this point had as part of the moving bike
      double vx0 = ride.v - ride.vth * (w.y - ride.z), vy0 = ride.vz + ride.vth * (w.x - ride.X);
      // the nearer the front, the more the boulder stops it; the back of the bike carries on into the wreck
      double d = clampd(piv.x / FRONT.x, 0, 1), keep = lerp(0.7, p.id == FRONTWHEEL ? -0.05 : 0.12, d);
      p.b.x = w.x; p.b.y = w.y; p.b.a = ride.th; p.b.hull = hull; p.b.I = (rr * 0.55) * (rr * 0.55);
      p.b.vx = vx0 * keep + (rnd() - 0.5) * 700;
      p.b.vy = std::min(0.0, vy0) * 0.3 + 250 + rnd() * (500 + 700 * d);  // a small jolt up, then gravity takes it
      double r1 = rnd(), r2 = rnd();
      p.b.w = ride.vth * 0.5 - (0.8 + r1 * 2.6) * (r2 < 0.8 ? 1 : -1) * (p.local ? 0.6 : 1);
    }
  }
  if (s == REASSEMBLE) {  // the bike rebuilds on the flat run-out past the boulder
    for (Part& p : parts) { p.fx = p.b.x; p.fy = p.b.y; p.fa = p.b.a; p.target = jsRound(p.b.a / (PI * 2)) * PI * 2; }
    camFrom = {camX, camZ};
    Seg* T = ride.trail; ride.X = T->start + T->len + runout(T) + 1900;
    rideReset(groundH(ride.X)); P = pose();
  }
}
std::vector<Saver::Debris> Saver::burst(double x, double y, int n, bool rock) {  // rock chips off the boulder, or bolts and shards off the bike
  std::vector<Debris> out;
  for (int i = 0; i < n; i++) {
    double ang = PI * (0.05 + rnd() * 0.9), sp = (rock ? 2500 : 1500) + rnd() * (rock ? 6500 : 4500);
    bool big = rock && rnd() < 0.35;
    Debris d{};
    d.x = x; d.y = y; d.vx = std::cos(ang) * sp * (rnd() < 0.3 ? -0.6 : 1); d.vy = std::sin(ang) * sp; d.a = rnd() * 6.28; d.w = (rnd() - 0.5) * 30;
    d.r = big ? 18 + rnd() * 30 : rock ? 5 + rnd() * 10 : 4 + rnd() * 8; d.n = 3 + (int)std::floor(rnd() * 3); d.shard = !big && rnd() < (rock ? 0.5 : 0.7);
    out.push_back(d);
  }
  return out;
}
void Saver::debrisStep(double h) {
  for (Debris& d : debris) {
    d.life += h; d.vy -= GRAV * h; d.x += d.vx * h; d.y += d.vy * h; d.a += d.w * h;
    double g = groundH(d.x);
    if (d.y - d.r * 0.5 < g) { d.y = g + d.r * 0.5; if (d.vy < 0) d.vy *= -0.3; d.vx *= 0.6; d.w *= 0.6; }
  }
}
// loose parts: gravity, then push out of the ground with a bouncy, frictional contact at the lowest point
void Saver::crashStep(double h) {
  for (Part& p : parts) {
    Body& b = p.b;
    b.vy -= GRAV * h; b.x += b.vx * h; b.y += b.vy * h; b.a += b.w * h;
    double c = std::cos(b.a), sn = std::sin(b.a), pen = 0, rx = 0, ry = 0;
    for (V2 q : b.hull) {
      double qx = q.x * c - q.y * sn, qy = q.x * sn + q.y * c, d = b.y + qy - groundH(b.x + qx);
      if (d < pen) { pen = d; rx = qx; ry = qy; }
    }
    if (pen < 0) {
      b.y -= pen;
      double vcy = b.vy + b.w * rx;
      if (vcy < 0) {
        double j = -1.4 * vcy / (1 + rx * rx / b.I);
        b.vy += j; b.w += rx * j / b.I;
        double vt = b.vx - b.w * ry, lim = 0.85 * j;
        double jt = clampd(-vt / (1 + ry * ry / b.I), -lim, lim);
        b.vx += jt; b.w -= ry * jt / b.I;
      }
      bool round = p.id == ROTORF || p.id == ROTORR || p.id == CASSETTE || p.id == FRONTWHEEL || p.id == REARWHEEL;  // would roll away
      b.vx *= 1 - (round ? 6 : 4) * h; b.w *= 1 - (round ? 9 : 4) * h;  // dig in, scrape and settle
    }
  }
}

/* ---------- mapping ---------- */
V2 Saver::homeScreen(V2 h) const { return {A.cx + (h.x - BIKE_C.x) * S + shake.x, A.cy - (h.y - BIKE_C.y) * S + shake.y}; }
// world (mm) -> screen through the camera; bike space -> world through the frame's position, heave and pitch
V2 Saver::ws(double wx, double wy) const { return homeScreen({wx - camX + COMB.x, wy - camZ + COMB.y}); }
V2 Saver::bikeWorld(V2 p) const {
  double c = std::cos(ride.th), s = std::sin(ride.th), dx = p.x - COMB.x, dy = p.y - COMB.y;
  return {ride.X + dx * c - dy * s, ride.z + dx * s + dy * c};
}
V2 Saver::bikeToScreen(V2 p) const { V2 w = bikeWorld(p); return ws(w.x, w.y); }

/* ---------- drawing helpers (millimetres, bike space, y up) ---------- */
void Saver::seg(V2 a, V2 b, double w, Col c) { ctx.strokeStyle = c; ctx.lineWidth = w; ctx.beginPath(); ctx.moveTo(a); ctx.lineTo(b); ctx.stroke(); }
void Saver::ring(V2 c, double r, double w, Col col) { ctx.strokeStyle = col; ctx.lineWidth = w; ctx.beginPath(); ctx.arc(c.x, c.y, r, 0, PI * 2); ctx.stroke(); }
// solid outline shape: filled with ground (hides what is behind), edged with object line
template <class F> void Saver::shape(F build, double edge) {
  ctx.beginPath(); build(); ctx.fillStyle = GROUND; ctx.fill(true); ctx.strokeStyle = LINE; ctx.lineWidth = px(edge); ctx.stroke();
}
void Saver::circ(V2 c, double r, double edge) { shape([&] { ctx.arc(c.x, c.y, r, 0, PI * 2); }, edge); }
// tubes drawn as two edges: wide object line, then ground core
void Saver::tubes(const std::vector<Tube>& list, bool round) {
  ctx.roundCap = round;
  for (const Tube& t : list) seg(t.a, t.b, t.w + px(2.6), LINE);
  for (const Tube& t : list) seg(t.a, t.b, t.w, GROUND);
  ctx.roundCap = true;
}
template <class F> void Saver::strokeOutlined(F build, double w) {
  ctx.beginPath(); build(); ctx.strokeStyle = LINE; ctx.lineWidth = w + px(2.4); ctx.stroke();
  ctx.strokeStyle = GROUND; ctx.lineWidth = w; ctx.stroke();
}
void Saver::centerMark(double r) {
  ctx.save(); ctx.setLineDash({px(DASH_CENTER[0]), px(DASH_CENTER[1]), px(DASH_CENTER[2]), px(DASH_CENTER[3])});
  ctx.strokeStyle = CENTER; ctx.lineWidth = px(0.8);
  ctx.beginPath(); ctx.moveTo(-r, 0); ctx.lineTo(r, 0); ctx.moveTo(0, -r); ctx.lineTo(0, r); ctx.stroke(); ctx.restore();
}

/* ---------- the parts ---------- */
void Saver::drawWheel(bool front) {
  shape([&] { ctx.arc(0, 0, 382, 0, PI * 2); ctx.arc(0, 0, 318, 0, PI * 2, true); }, 1.4);
  ctx.strokeStyle = LINE; ctx.lineWidth = px(1); ctx.beginPath();
  int n = front ? 46 : 42;
  for (int i = 0; i < n; i++) {
    double a = wheelRot + i * PI * 2 / n, r1 = i % 2 ? 392 : 388, da = 0.022;
    ctx.moveTo(std::cos(a - da) * 382, std::sin(a - da) * 382); ctx.lineTo(std::cos(a - da) * r1, std::sin(a - da) * r1);
    ctx.lineTo(std::cos(a + da) * r1, std::sin(a + da) * r1); ctx.lineTo(std::cos(a + da) * 382, std::sin(a + da) * 382);
  }
  ctx.stroke();
  ring({0, 0}, 305, px(1.1), LINE); ring({0, 0}, 292, px(0.8), THIN);
  ctx.strokeStyle = THIN; ctx.lineWidth = px(0.6); ctx.beginPath();
  for (int i = 0; i < 32; i++) {
    double a = wheelRot + i * PI * 2 / 32, b = a + (i % 2 ? 0.32 : -0.32);
    ctx.moveTo(std::cos(a) * 34, std::sin(a) * 34); ctx.lineTo(std::cos(b) * 291, std::sin(b) * 291);
  }
  ctx.stroke();
  circ({0, 0}, 34, 1.3); circ({0, 0}, 11, 1);
  centerMark(420);
}
void Saver::drawRotor(double R) {
  ctx.save(); ctx.rotate(wheelRot);
  shape([&] { ctx.arc(0, 0, R, 0, PI * 2); ctx.arc(0, 0, R * 0.78, 0, PI * 2, true); }, 1.1);
  for (int i = 0; i < 24; i++) { double a = i * PI * 2 / 24; ring({std::cos(a) * R * 0.89, std::sin(a) * R * 0.89}, R * 0.034, px(0.7), THIN); }
  for (int i = 0; i < 6; i++) {
    double a = i * PI / 3;
    strokeOutlined([&] {
      ctx.moveTo(std::cos(a) * R * 0.24, std::sin(a) * R * 0.24);
      ctx.quadraticCurveTo(std::cos(a + 0.35) * R * 0.55, std::sin(a + 0.35) * R * 0.55, std::cos(a + 0.5) * R * 0.79, std::sin(a + 0.5) * R * 0.79);
    }, R * 0.06);
  }
  circ({0, 0}, R * 0.24, 1.1);
  for (int i = 0; i < 6; i++) { double a = i * PI / 3 + 0.5; ring({std::cos(a) * R * 0.17, std::sin(a) * R * 0.17}, R * 0.025, px(0.7), LINE); }
  ctx.restore();
  centerMark(R * 1.15);
}
void Saver::drawCassette() {
  ctx.save(); ctx.rotate(wheelRot);
  circ({0, 0}, 52 * 2.02 + 5, 1.2);
  for (int i = 11; i >= 0; i--) ring({0, 0}, COGS[i] * 2.02, px(i % 2 ? 0.7 : 1), i % 2 ? THIN : LINE);
  ctx.strokeStyle = LINE; ctx.lineWidth = px(0.8); ctx.beginPath();
  for (int k = 0; k < 52; k++) { double a = k * PI * 2 / 52, r = 52 * 2.02 + 5; ctx.moveTo(std::cos(a) * r, std::sin(a) * r); ctx.lineTo(std::cos(a) * (r + 5), std::sin(a) * (r + 5)); }
  ctx.stroke();
  circ({0, 0}, 15, 1); ctx.restore();
  centerMark(130);
}
void Saver::drawCrankset() {
  const int N = 32; double r = N * 2.02;
  ctx.save(); ctx.rotate(crank);
  shape([&] {
    for (int k = 0; k < N * 2; k++) { double a = k * PI / N, rr = k % 2 ? r - 4 : r + 6; ctx.lineTo(std::cos(a) * rr, std::sin(a) * rr); }
    ctx.closePath(); ctx.moveTo(r * 0.62, 0); ctx.arc(0, 0, r * 0.62, 0, PI * 2, true);
  }, 1.2);
  ring({0, 0}, r - 10, px(0.7), THIN);
  for (int i = 0; i < 4; i++) { double a = i * PI / 2 + 0.4; ring({std::cos(a) * r * 0.72, std::sin(a) * r * 0.72}, 4.5, px(0.8), LINE); }
  ctx.restore();
  V2 end{std::cos(crank) * 170, std::sin(crank) * 170};
  tubes({{{0, 0}, end, 30}});
  circ({0, 0}, 24, 1.2); circ({0, 0}, 10, 0.9);
  shape([&] { ctx.roundRect(end.x - 55, end.y - 10, 110, 20, 5); }, 1.2);
  ctx.strokeStyle = LINE; ctx.lineWidth = px(0.9); ctx.beginPath();
  for (int k = -2; k <= 2; k++) { ctx.moveTo(end.x + k * 22, end.y + 10); ctx.lineTo(end.x + k * 22, end.y + 17); ctx.moveTo(end.x + k * 22, end.y - 10); ctx.lineTo(end.x + k * 22, end.y - 17); }
  ctx.stroke(); ring(end, 5, px(0.8), THIN);
  centerMark(90);
}
void Saver::drawDerailleur() {
  V2 up{15, -75}, lo{40, -145};
  ctx.save(); ctx.translate(PIVOT.x, PIVOT.y); ctx.rotate(P.phi); ctx.translate(-PIVOT.x, -PIVOT.y);  // rides on the swingarm
  tubes({{{18, -12}, {62, -36}, 20}});
  ctx.save(); ctx.translate(62, -50); ctx.rotate(-0.5);
  shape([&] { ctx.roundRect(-28, -18, 56, 36, 8); }, 1.2); ring({0, 0}, 7, px(0.8), LINE); ctx.restore();
  tubes({{{60, -60}, up, 12}});
  U3 d = unit(up, lo);
  ctx.save(); ctx.translate(up.x, up.y); ctx.rotate(std::atan2(d.y, d.x));
  shape([&] { ctx.roundRect(-22, -22, d.l + 44, 44, 22); }, 1.2); ctx.restore();
  for (V2 j : {up, lo}) {
    circ(j, 15, 1);
    ctx.strokeStyle = LINE; ctx.lineWidth = px(0.8); ctx.beginPath();
    for (int k = 0; k < 12; k++) { double a = crank * 5 + k * PI / 6; ctx.moveTo(j.x + std::cos(a) * 15, j.y + std::sin(a) * 15); ctx.lineTo(j.x + std::cos(a) * 19, j.y + std::sin(a) * 19); }
    ctx.stroke(); ring(j, 4, px(0.8), LINE);
  }
  ctx.restore();
}
void Saver::drawFrame() {
  tubes({{BB, ST, 38}, {ST, TT_END, 36}, {BB, DT_END, 56}, {along(HT_TOP, HDIR, -6), along(HT_BOT, HDIR, 6), 50}, {{ST.x + 10, ST.y - 40}, {ST.x + 120, ST.y - 20}, 26}});
  circ(BB, 30, 1.3); ring(BB, 10, px(0.8), THIN);
  ring(PIVOT, 13, px(1), LINE); ring(ROCK[1], 11, px(1), LINE); ring(SHOCK_A, 13, px(1), LINE);
  ctx.save(); ctx.setLineDash({px(6), px(4)});
  U3 dd = unit(BB, DT_END); seg(along(BB, {dd.x, dd.y}, 70), along(BB, {dd.x, dd.y}, 560), px(0.8), THIN); ctx.restore();  // hidden line: internal cable
}
void Saver::drawSwingarm() {
  V2 R = P.rear, r0 = P.rock0, r2 = P.rock2;
  tubes({{PIVOT, R, 34}, {R, r0, 28}});
  shape([&] { ctx.moveTo(r0); ctx.lineTo(ROCK[1]); ctx.lineTo(r2); ctx.closePath(); }, 1.3);
  for (V2 p : {r0, ROCK[1], r2}) ring(p, 9, px(1), LINE);
  circ(R, 26, 1.3); circ(PIVOT, 13, 1);
}
void Saver::drawShock() {
  U3 u = unit(SHOCK_A, P.rock2); double L = u.l, L0 = SHOCK_L0, perch = L - L0 * 0.14;
  ctx.save(); ctx.translate(SHOCK_A.x, SHOCK_A.y); ctx.rotate(std::atan2(u.y, u.x));
  shape([&] { ctx.roundRect(14, -17, L0 * 0.55, 34, 6); }, 1.3);
  shape([&] { ctx.rect(L0 * 0.55, -7, std::max(2.0, perch - L0 * 0.55), 14); }, 0.9);
  shape([&] { ctx.roundRect(perch, -20, 22, 40, 4); }, 1.2);
  shape([&] { ctx.roundRect(L0 * 0.12, -22, 16, 44, 3); }, 1.2);
  double s0 = L0 * 0.18, s1 = perch; const int coils = 8;
  ctx.lineWidth = px(1.1); ctx.strokeStyle = LINE; ctx.beginPath();
  for (int k = 0; k <= coils * 2; k += 2) { double x = lerp(s0, s1, k / (coils * 2.0)), x2 = lerp(s0, s1, (k + 1) / (coils * 2.0)); ctx.moveTo(x, -30); ctx.lineTo(std::min(x2, s1), 30); }
  ctx.stroke();
  ctx.strokeStyle = THIN; ctx.lineWidth = px(0.7); ctx.setLineDash({px(4), px(3)}); ctx.beginPath();  // hidden back of each coil
  for (int k = 1; k <= coils * 2; k += 2) { double x = lerp(s0, s1, k / (coils * 2.0)), x2 = lerp(s0, s1, (k + 1) / (coils * 2.0)); ctx.moveTo(x, 30); ctx.lineTo(std::min(x2, s1), -30); }
  ctx.stroke(); ctx.setLineDash({});
  circ({0, 0}, 13, 1.1); circ({L, 0}, 13, 1.1);
  ctx.save();
  ctx.setLineDash({px(DASH_CENTER[0]), px(DASH_CENTER[1]), px(DASH_CENTER[2]), px(DASH_CENTER[3])}); seg({-30, 0}, {L + 30, 0}, px(0.8), CENTER); ctx.restore();
  ctx.restore();
}
void Saver::drawFork() {
  U3 u = unit(CROWN, FRONT); V2 dir{u.x, u.y}, n{-dir.y, dir.x}; double L = u.l, c = P.tf;
  tubes({{CROWN, HT_TOP, 30}}, false);
  V2 lowTop = along(CROWN, dir, L * 0.4 - c), axle = along(FRONT, dir, -c);
  tubes({{along(CROWN, dir, 18), along(lowTop, dir, 12), 36}}, false);
  tubes({{lowTop, along(axle, dir, -8), 54}}, false);
  seg(along(along(lowTop, dir, 14), n, -27), along(along(lowTop, dir, 14), n, 27), px(0.8), THIN);  // seal
  tubes({{along(along(CROWN, n, -48), dir, 8), along(along(CROWN, n, 48), dir, 8), 32}}, false);
  ctx.save(); V2 b = along(axle, dir, -70); ctx.translate(b.x, b.y); ctx.rotate(std::atan2(u.y, u.x));
  shape([&] { ctx.roundRect(-30, -44, 60, 26, 6); }, 1.1); ctx.restore();
  circ(axle, 22, 1.2);
  ctx.save(); ctx.setLineDash({px(DASH_CENTER[0]), px(DASH_CENTER[1]), px(DASH_CENTER[2]), px(DASH_CENTER[3])});
  seg(along(HT_TOP, HDIR, -60), along(CROWN, dir, L * 0.95), px(0.8), CENTER); ctx.restore();
}
void Saver::drawCockpit() {
  V2 stemEnd = FX({1040, 585});
  tubes({{HT_TOP, stemEnd, 34}});
  circ(stemEnd, 22, 1.3); ring(stemEnd, 15, px(0.8), THIN);
  tubes({{stemEnd, FX({1002, 612}), 34}});
  tubes({{FX({1028, 593}), FX({1036, 589}), 38}}, false);
  seg(FX({1022, 590}), FX({1102, 566}), px(1.4), LINE);
  ring(FX({1026, 594}), 9, px(1), LINE);
}
void Saver::drawDropper() {
  V2 outerTop = along(ST, POST_DIR, 70), top = along(ST, POST_DIR, 185);
  tubes({{along(ST, POST_DIR, -10), outerTop, 34}}, false);
  tubes({{outerTop, top, 27}}, false);
  tubes({{along(ST, POST_DIR, -6), along(ST, POST_DIR, 16), 46}}, false);
  for (double d : {100.0, 140.0}) { V2 p = along(ST, POST_DIR, d), q{POST_DIR.y, -POST_DIR.x}; seg(along(p, q, -10), along(p, q, 10), px(0.7), THIN); }
  ctx.save(); ctx.translate(top.x, top.y);
  shape([&] { ctx.moveTo(-150, 18); ctx.quadraticCurveTo(-150, 46, -115, 44); ctx.quadraticCurveTo(0, 30, 150, 28); ctx.quadraticCurveTo(170, 24, 160, 14); ctx.quadraticCurveTo(20, 10, -130, 6); ctx.closePath(); }, 1.3);
  seg({-95, 8}, {85, 10}, px(0.8), THIN);
  ctx.restore();
}
void Saver::drawChain(double alpha) {
  double ringR = 32 * 2.02, cogR = 21 * 2.02; V2 R = P.rear;
  V2 up = rotAbout({15, -75}, PIVOT, P.phi), lo = rotAbout({40, -145}, PIVOT, P.phi);
  std::vector<V2> pts{{R.x, R.y + cogR}, {BB.x, BB.y + ringR}};
  auto arcPts = [&](V2 c, double r, double a0, double a1, int n) { for (int i = 0; i <= n; i++) { double a = lerp(a0, a1, (double)i / n); pts.push_back({c.x + std::cos(a) * r, c.y + std::sin(a) * r}); } };
  arcPts(BB, ringR, PI / 2, -PI / 2, 12); arcPts(lo, 15, -PI / 2, -PI, 5); arcPts(up, 15, -0.2, PI * 0.6, 5); arcPts(R, cogR, -PI / 2, -PI * 1.5, 14);
  auto path = [&] { ctx.moveTo(pts[0]); for (size_t i = 1; i < pts.size(); i++) ctx.lineTo(pts[i]); };
  ctx.globalAlpha = alpha; ctx.roundJoin = true;
  strokeOutlined(path, 9);
  ctx.beginPath(); path(); ctx.setLineDash({px(1), 12.7 - px(1)}); ctx.lineDashOffset = crank * ringR;
  ctx.strokeStyle = LINE; ctx.lineWidth = 9; ctx.stroke();
  ctx.setLineDash({}); ctx.lineDashOffset = 0; ctx.globalAlpha = 1;
}
void Saver::drawPart(int id) {
  switch (id) {
    case FRAME: drawFrame(); break; case SWINGARM: drawSwingarm(); break; case SHOCK: drawShock(); break; case FORK: drawFork(); break;
    case FRONTWHEEL: drawWheel(true); break; case REARWHEEL: drawWheel(false); break;
    case ROTORF: drawRotor(101.5); break; case ROTORR: drawRotor(90); break; case CASSETTE: drawCassette(); break;
    case CRANKSET: drawCrankset(); break; case DERAILLEUR: drawDerailleur(); break; case COCKPIT: drawCockpit(); break; default: drawDropper();
  }
}

/* ---------- labels ---------- */
void Saver::balloon(V2 p0, V2 bc, double br, int num, const std::string& name, double al, double fs) {
  U3 d = unit(p0, bc);
  ctx.strokeStyle = dimA(0.85 * al); ctx.fillStyle = dimA(al); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(p0); ctx.lineTo(bc.x - d.x * br, bc.y - d.y * br); ctx.stroke();
  ctx.beginPath(); ctx.arc(p0.x, p0.y, 2.2, 0, PI * 2); ctx.fill();
  ctx.fillStyle = GROUND; ctx.beginPath(); ctx.arc(bc.x, bc.y, br, 0, PI * 2); ctx.fill(); ctx.stroke();
  ctx.fillStyle = dimA(al); ctx.fontSize = fs; ctx.fontWeight = 500; ctx.textAlign = Align::Center; ctx.textBaseline = Baseline::Middle;
  ctx.fillText(std::to_string(num), bc.x, bc.y + 0.5);
  if (!name.empty()) {
    bool left = d.x < -0.2; ctx.textAlign = left ? Align::Right : Align::Left; ctx.fontSize = fs - 1; ctx.fontWeight = 400;
    double tx = bc.x + (left ? -br - 5 : br + 5), tw = ctx.measureText(name);
    ctx.fillStyle = GROUND; ctx.fillRect(left ? tx - tw - 2 : tx - 2, bc.y - fs / 2 - 1, tw + 4, fs + 2);
    ctx.fillStyle = dimA(al); ctx.fillText(name, tx, bc.y + 0.5);
  }
}
void Saver::drawAssemblyCallouts(double al) {
  if (al < 0.01) return;
  double fs = W < 600 ? 9 : 11, br = fs + 1;
  if (!wide) {  // narrow screens: compact balloons with names
    double k = std::min(1.0, S / 0.42);
    for (const Part& p : parts) {
      Tag t = tagFor(p.id); double ang = t.deg * PI / 180;
      V2 p0 = bikeToScreen(t.anchor); double L = t.len * k + br + 14;
      balloon(p0, {p0.x + std::cos(ang) * L, p0.y - std::sin(ang) * L}, br, p.item, upper(p.name), al, fs);
    }
    return;
  }
  // wide screens: two label columns either side of the bike, sorted top to bottom so leaders don't cross
  struct Item { const Part* p; V2 a; double ax; };
  std::vector<Item> items;
  for (const Part& p : parts) { Tag t = tagFor(p.id); items.push_back({&p, bikeToScreen(t.anchor), t.anchor.x}); }
  std::stable_sort(items.begin(), items.end(), [](const Item& m, const Item& n) { return m.ax < n.ax; });
  std::vector<Item> cols[2] = {std::vector<Item>(items.begin(), items.begin() + 7), std::vector<Item>(items.begin() + 7, items.end())};
  double bl = bikeToScreen({-420, 0}).x, brx = bikeToScreen({FRONT.x + 420, 0}).x;
  double y0 = A.y + A.h * 0.1, y1 = A.y + A.h * 0.86;
  for (int ci = 0; ci < 2; ci++) {
    auto& col = cols[ci]; bool left = ci == 0;
    std::stable_sort(col.begin(), col.end(), [](const Item& m, const Item& n) { return m.a.y < n.a.y; });
    double bx = left ? bl - 18 - br : brx + 18 + br;
    for (size_t i = 0; i < col.size(); i++) {
      const Item& it = col[i];
      double y = col.size() == 1 ? (y0 + y1) / 2 : lerp(y0, y1, (double)i / (col.size() - 1));
      V2 shoulder{bx + (left ? br + 16 : -br - 16), y};
      ctx.strokeStyle = dimA(0.75 * al); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(it.a); ctx.lineTo(shoulder); ctx.lineTo(bx + (left ? br : -br), y); ctx.stroke();
      ctx.fillStyle = dimA(al); ctx.beginPath(); ctx.arc(it.a.x, it.a.y, 2.4, 0, PI * 2); ctx.fill();
      ctx.fillStyle = GROUND; ctx.beginPath(); ctx.arc(bx, y, br, 0, PI * 2); ctx.fill(); ctx.stroke();
      ctx.fillStyle = dimA(al); ctx.fontSize = fs; ctx.fontWeight = 500; ctx.textAlign = Align::Center; ctx.textBaseline = Baseline::Middle;
      ctx.fillText(std::to_string(it.p->item), bx, y + 0.5);
      double tx = bx + (left ? -br - 8 : br + 8);
      ctx.textAlign = left ? Align::Right : Align::Left;
      ctx.textBaseline = Baseline::Bottom; ctx.fontSize = fs; ctx.fontWeight = 500; ctx.fillStyle = lineA(al);
      ctx.fillText(upper(it.p->name), tx, y - 1);
      ctx.textBaseline = Baseline::Top; ctx.fontSize = fs - 1; ctx.fontWeight = 400; ctx.fillStyle = lineA(0.6 * al);
      ctx.fillText(it.p->spec, tx, y + 2);
    }
  }
}

/* ---------- terrain drawing ---------- */
void Saver::drawTerrain(double al) {
  if (al < 0.01) return;
  std::vector<V2> pts;
  for (double sx = A.x - 4; sx <= A.x + A.w + 4; sx += 3) {
    double qx = (sx - A.cx) / S + BIKE_C.x, wx = qx + camX - COMB.x;
    pts.push_back({sx, A.cy - (groundH(wx) - camZ + COMB.y - BIKE_C.y) * S});
  }
  double bottom = A.y + A.h;
  ctx.save(); ctx.globalAlpha = al;
  ctx.beginPath(); ctx.moveTo(pts[0].x, bottom); for (V2 p : pts) ctx.lineTo(p); ctx.lineTo(pts.back().x, bottom); ctx.closePath();
  ctx.fillStyle = GROUND; ctx.fill();
  ctx.save();
  // section hatching, fixed to the ground: each 45° line is cut where it leaves the soil (the page clips to the
  // ground outline instead; computing the cut keeps Direct2D from rendering an off-screen layer every frame)
  double step = 11, off = std::fmod(std::fmod(camX * S, step) + step, step);
  auto groundY = [&](double x) {  // screen y of the surface at screen x (the profile is sampled every 3 px)
    double f = (x - pts[0].x) / 3; if (f <= 0) return pts[0].y; size_t i = (size_t)f; if (i + 1 >= pts.size()) return pts.back().y;
    return lerp(pts[i].y, pts[i + 1].y, f - i);
  };
  ctx.strokeStyle = THIN; ctx.globalAlpha = al * 0.55; ctx.lineWidth = 1; ctx.beginPath();
  for (double x = A.x - A.h - off; x < A.x + A.w + step; x += step) {
    // walk up the line (x + t, bottom - t); depth > 0 while it is under the surface
    auto depth = [&](double t) { return (bottom - t) - groundY(x + t); };
    double t0 = std::max(0.0, A.x - 4 - x), t1 = std::min(A.h, A.x + A.w + 4 - x);
    if (t1 <= t0) continue;
    double prev = t0, dp = depth(t0), start = dp > 0 ? t0 : -1;
    for (double t = t0 + 3; ; t += 3) {
      bool last = t >= t1; if (last) t = t1;
      double d = depth(t);
      if ((dp > 0) != (d > 0)) {  // crossed the surface between prev and t
        double tc = prev + (t - prev) * dp / (dp - d);
        if (d > 0) start = tc; else { ctx.moveTo(x + start, bottom - start); ctx.lineTo(x + tc, bottom - tc); start = -1; }
      }
      prev = t; dp = d;
      if (last) break;
    }
    if (start >= 0) { ctx.moveTo(x + start, bottom - start); ctx.lineTo(x + t1, bottom - t1); }
  }
  ctx.stroke();
  // buried stones in the soil section, fixed to the ground
  double w0 = (A.x - A.cx) / S + BIKE_C.x + camX - COMB.x, w1 = w0 + A.w / S, cell = 330;
  ctx.globalAlpha = al; ctx.lineWidth = 1; ctx.strokeStyle = THIN; ctx.fillStyle = GROUND;
  for (double j = std::floor(w0 / cell) - 1; j <= w1 / cell + 1; j++) {
    double q = j * 3.7 + 50; if (hs2(q) < 0.35) continue;
    double r = 18 + std::pow(hs2(q + 0.2), 2) * 120, cx = j * cell + hs2(q + 0.4) * cell;
    double cy = groundH(cx) - r * 0.6 - hs2(q + 0.6) * 700, rot = hs2(q + 0.9) * 6.28; int n = 6 + (int)std::floor(hs2(q + 0.8) * 4);
    // with no clip to the ground outline, leave out stones that would poke through the surface
    if (cy + r * 0.8 > std::min({groundH(cx - r * 1.25), groundH(cx), groundH(cx + r * 1.25)}) - 4) continue;
    ctx.beginPath();
    for (int k = 0; k < n; k++) {
      double ang = rot + (double)k / n * PI * 2, rr = r * (0.65 + 0.35 * hs2(q + k * 1.31));
      V2 p = ws(cx + std::cos(ang) * rr * 1.25, cy + std::sin(ang) * rr * 0.8);
      if (k) ctx.lineTo(p); else ctx.moveTo(p);
    }
    ctx.closePath(); ctx.fill(); ctx.stroke();
  }
  // the boulder: solid rock, outlined, with a buried base and a couple of fracture lines
  for (Seg& T : ride.segs) {
    double lip = T.start + T.len, BX = T.bx ? T.bx : BOULDER_X;
    if (lip + BX > w1 + 500 || lip + BX + 930 < w0 - 500) continue;
    double bot = trailH(T.len, &T) - T.drop;
    auto base = [&](double v) { trailSeed = T.seed; return T.y0 + belowCliff(T.len + v, v, bot, &T); };
    std::vector<V2> poly;
    for (V2 b : BOULDER) { double v = BX + b.x; poly.push_back(ws(lip + v, base(v) + boulderTop(v, &T))); }
    for (int i = 12; i >= 0; i--) { double dx = BOULDER[i].x, v = BX + dx; poly.push_back(ws(lip + v, base(v) - 140 - 60 * std::sin(PI * dx / 930))); }
    ctx.globalAlpha = al; ctx.fillStyle = GROUND; ctx.strokeStyle = LINE; ctx.lineWidth = 1.3;
    ctx.beginPath(); for (size_t i = 0; i < poly.size(); i++) { if (i) ctx.lineTo(poly[i]); else ctx.moveTo(poly[i]); } ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.strokeStyle = THIN; ctx.lineWidth = 1; ctx.beginPath();
    auto at = [&](double dx, double dy) { double v = BX + dx; return ws(lip + v, base(v) + dy); };
    ctx.moveTo(at(250, 380)); ctx.lineTo(at(300, 250)); ctx.lineTo(at(280, 120)); ctx.lineTo(at(340, 10));
    ctx.moveTo(at(600, 430)); ctx.lineTo(at(560, 300)); ctx.lineTo(at(620, 180));
    ctx.stroke();
  }
  ctx.restore();
  ctx.globalAlpha = al; ctx.strokeStyle = LINE; ctx.lineWidth = 1.6; ctx.roundJoin = true;
  ctx.beginPath(); for (size_t i = 0; i < pts.size(); i++) { if (i) ctx.lineTo(pts[i]); else ctx.moveTo(pts[i]); } ctx.stroke();
  ctx.fontSize = W < 600 ? 9 : 10; ctx.fontWeight = 500; ctx.textAlign = Align::Left; ctx.textBaseline = Baseline::Middle;
  if (tiny) { ctx.restore(); return; }
  std::string t = "SECTION A–A · TRAIL PROFILE"; double tw = ctx.measureText(t) + 10, ly = bottom - 14;
  ctx.fillStyle = GROUND; ctx.fillRect(A.x + 6, ly - 8, tw, 16);
  ctx.fillStyle = DIM; ctx.fillText(t, A.x + 11, ly);
  ctx.restore();
}
void Saver::drawDebris() {
  if (state != CRASH || debris.empty()) return;
  double al = 1 - smooth(DUR_CRASH - 1.2, DUR_CRASH - 0.2, st);
  ctx.save(); ctx.globalAlpha = al; ctx.lineWidth = 1; ctx.roundJoin = true;
  for (const Debris& d : debris) {
    V2 c = ws(d.x, d.y); double r = d.r * S;
    ctx.beginPath();
    if (d.shard) {
      ctx.moveTo(c.x - std::cos(d.a) * r * 1.6, c.y - std::sin(d.a) * r * 1.6); ctx.lineTo(c.x + std::cos(d.a) * r * 1.6, c.y + std::sin(d.a) * r * 1.6);
      ctx.strokeStyle = THIN; ctx.stroke(); continue;
    }
    for (int k = 0; k < d.n; k++) {
      double t = d.a + (double)k / d.n * 6.28, rr = r * (0.7 + 0.3 * ((k * 7) % 3) / 2);
      if (k) ctx.lineTo(c.x + std::cos(t) * rr, c.y + std::sin(t) * rr); else ctx.moveTo(c.x + std::cos(t) * rr, c.y + std::sin(t) * rr);
    }
    ctx.closePath(); ctx.fillStyle = GROUND; ctx.fill(); ctx.strokeStyle = LINE; ctx.stroke();
  }
  ctx.restore();
}

/* ---------- sheet: border, zones, grid ---------- */
void Saver::drawSheet() {
  ctx.fillStyle = GROUND; ctx.fillRect(0, 0, W, H);
  // model-space grid inside the viewport
  ctx.save(); ctx.beginPath(); ctx.rect(A.x, A.y, A.w, A.h); ctx.clip();
  double gs = 40; ctx.beginPath();
  for (double x = A.x + std::fmod(A.w / 2, gs); x < A.x + A.w; x += gs) { ctx.moveTo(x, A.y); ctx.lineTo(x, A.y + A.h); }
  for (double y = A.y + std::fmod(A.h / 2, gs); y < A.y + A.h; y += gs) { ctx.moveTo(A.x, y); ctx.lineTo(A.x + A.w, y); }
  ctx.lineWidth = 1; ctx.strokeStyle = GRID; ctx.stroke(); ctx.restore();
  // border with zones (ISO 5457: numbers left to right, letters top to bottom)
  ctx.strokeStyle = BORDER; ctx.lineWidth = 1; ctx.strokeRect(mo + 0.5, mo + 0.5, W - 2 * mo - 1, H - 2 * mo - 1);
  ctx.lineWidth = 1.6; ctx.strokeRect(mi, mi, W - 2 * mi, H - 2 * mi);
  if (tiny) return;
  int nc = (int)std::max(4.0, jsRound(W / 220)), nr = (int)std::max(3.0, jsRound(H / 220));
  ctx.fontSize = W < 600 ? 8 : 10; ctx.fontWeight = 500; ctx.fillStyle = DIM; ctx.textAlign = Align::Center; ctx.textBaseline = Baseline::Middle;
  ctx.lineWidth = 1; ctx.beginPath();
  for (int i = 0; i < nc; i++) {
    double x0 = mi + (W - 2 * mi) * i / nc, xm = mi + (W - 2 * mi) * (i + 0.5) / nc;
    if (i) { ctx.moveTo(x0, mo); ctx.lineTo(x0, mi); ctx.moveTo(x0, H - mi); ctx.lineTo(x0, H - mo); }
    ctx.fillText(std::to_string(i + 1), xm, (mo + mi) / 2); ctx.fillText(std::to_string(i + 1), xm, H - (mo + mi) / 2);
  }
  for (int j = 0; j < nr; j++) {
    double y0 = mi + (H - 2 * mi) * j / nr, ym = mi + (H - 2 * mi) * (j + 0.5) / nr; std::string L(1, (char)('A' + j));
    if (j) { ctx.moveTo(mo, y0); ctx.lineTo(mi, y0); ctx.moveTo(W - mi, y0); ctx.lineTo(W - mo, y0); }
    ctx.fillText(L, (mo + mi) / 2, ym); ctx.fillText(L, W - (mo + mi) / 2, ym);
  }
  ctx.stroke();
  // centring marks
  ctx.beginPath(); ctx.moveTo(W / 2, mo); ctx.lineTo(W / 2, mi + 8); ctx.moveTo(W / 2, H - mo); ctx.lineTo(W / 2, H - mi - 8);
  ctx.moveTo(mo, H / 2); ctx.lineTo(mi + 8, H / 2); ctx.moveTo(W - mo, H / 2); ctx.lineTo(W - mi - 8, H / 2); ctx.lineWidth = 1.6; ctx.stroke();
}

/* ---------- dimensions (screen space) ---------- */
void Saver::arrowHead(V2 p, V2 dir) {
  V2 n{-dir.y, dir.x}; double L = 8, w = 2.4;
  ctx.beginPath(); ctx.moveTo(p);
  ctx.lineTo(p.x - dir.x * L + n.x * w, p.y - dir.y * L + n.y * w);
  ctx.lineTo(p.x - dir.x * L - n.x * w, p.y - dir.y * L - n.y * w); ctx.closePath(); ctx.fill();
}
void Saver::dimText(const std::string& t, double x, double y, double al) {
  ctx.textAlign = Align::Center; ctx.textBaseline = Baseline::Middle;
  double w = ctx.measureText(t) + 8;
  ctx.fillStyle = GROUND; ctx.fillRect(x - w / 2, y - 7, w, 14);
  ctx.fillStyle = dimA(al); ctx.fillText(t, x, y);
}
void Saver::hDim(V2 P1, V2 P2, double y, const std::string& text, double al) {
  double s = (y - P1.y) > 0 ? 1 : (y - P1.y) < 0 ? -1 : 1;
  ctx.beginPath();
  ctx.moveTo(P1.x, P1.y + s * 6); ctx.lineTo(P1.x, y + s * 6);
  ctx.moveTo(P2.x, P2.y + s * 6); ctx.lineTo(P2.x, y + s * 6);
  ctx.moveTo(P1.x, y); ctx.lineTo(P2.x, y); ctx.stroke();
  arrowHead({P1.x, y}, {-1, 0}); arrowHead({P2.x, y}, {1, 0});
  dimText(text, (P1.x + P2.x) / 2, y, al);
}
void Saver::angDim(V2 Vt, double r, double a0, double a1, const std::string& text, double al) {
  V2 c = bikeToScreen(Vt); double R = r * S;
  ctx.beginPath();
  for (double a : {a0, a1}) { ctx.moveTo(c.x + std::cos(a) * 6, c.y - std::sin(a) * 6); ctx.lineTo(c.x + std::cos(a) * (R + 12), c.y - std::sin(a) * (R + 12)); }
  ctx.stroke();
  ctx.beginPath(); ctx.arc(c.x, c.y, R, -a1, -a0); ctx.stroke();
  arrowHead({c.x + std::cos(a0) * R, c.y - std::sin(a0) * R}, {-std::sin(a0), -std::cos(a0)});
  arrowHead({c.x + std::cos(a1) * R, c.y - std::sin(a1) * R}, {std::sin(a1), std::cos(a1)});
  double m = (a0 + a1) / 2; dimText(text, c.x + std::cos(m) * (R + 18), c.y - std::sin(m) * (R + 18), al);
}
void Saver::drawDims(double al) {
  if (al <= 0.01) return;
  ctx.save();
  ctx.fontSize = W < 600 ? 9 : 11; ctx.fontWeight = 500;
  auto colors = [&] { ctx.strokeStyle = dimA(al * 0.9); ctx.fillStyle = dimA(al); };
  colors(); ctx.lineWidth = 1;
  V2 R = bikeToScreen(P.rear), F = bikeToScreen(P.front), B = bikeToScreen(BB);
  hDim(R, F, bikeToScreen({0, -500}).y, std::to_string((int)(FRONT.x - REAR.x)), al);
  colors(); hDim(R, B, bikeToScreen({0, -430}).y, std::to_string((int)(BB.x - REAR.x)), al);
  colors(); angDim(HT_TOP, 120, PI - HA, PI, "65°", al);
  colors(); angDim(BB, 150, PI - SEAT_ANGLE * PI / 180, PI, std::to_string((int)jsRound(SEAT_ANGLE)) + "°", al);
  // diameter leader on the front wheel
  colors();
  double a = 0.75; V2 p0 = bikeToScreen({P.front.x + std::cos(a) * 382, P.front.y + std::sin(a) * 382}), p1{p0.x + 34, p0.y - 34};
  ctx.beginPath(); ctx.moveTo(p0); ctx.lineTo(p1); ctx.lineTo(p1.x + 26, p1.y); ctx.stroke();
  arrowHead(p0, {-std::sqrt(0.5), std::sqrt(0.5)});
  ctx.textAlign = Align::Left; ctx.textBaseline = Baseline::Middle; ctx.fillText("Ø764", p1.x + 30, p1.y);
  ctx.restore();
}

/* ---------- parts ---------- */
void Saver::drawParts(double a) {
  struct St { V2 pos; double rot; };
  St states[13];
  for (size_t i = 0; i < parts.size(); i++) {
    const Part& p = parts[i]; V2 pos; double rot;
    if (state == RIDE) { pos = bikeToScreen(partPivot(p)); rot = -ride.th; }
    else if (state == CRASH) { pos = ws(p.b.x, p.b.y); rot = -p.b.a; }
    else {
      V2 hw = bikeWorld(partPivot(p));
      pos = ws(lerp(p.fx, hw.x, a), lerp(p.fy, hw.y, a) + std::sin(PI * a) * 520);  // lift off the ground, arc home
      rot = -lerp(p.fa, p.target, a);
    }
    states[i] = {pos, rot};
  }
  ctx.save(); ctx.beginPath(); ctx.rect(A.x, A.y, A.w, A.h); ctx.clip();
  drawTerrain(1);
  for (int id : ORDER) {
    const Part& p = parts[id]; const St& s = states[id];
    ctx.save();
    ctx.translate(s.pos.x, s.pos.y); ctx.rotate(s.rot); ctx.scale(S, -S);
    if (!p.local) ctx.translate(-p.home.x, -p.home.y);
    U_ = 1 / S;
    ctx.roundCap = true; ctx.roundJoin = true;
    if (id == CRANKSET && state != CRASH && a > 0.85) { ctx.save(); ctx.translate(-BB.x, -BB.y); drawChain((a - 0.85) / 0.15); ctx.restore(); }
    drawPart(id);
    ctx.restore();
  }
  drawDebris();
  // impact marker where the front wheel hit
  double tm = haveCrash ? time - crash.t0 : 99;
  if (state == CRASH && tm < 2.6) {
    double al = 1 - tm / 2.6, r = 14 + tm * 110;
    V2 c = ws(crash.w.x, crash.w.y);
    ctx.strokeStyle = centerA(al); ctx.lineWidth = 1;
    ctx.setLineDash({6, 4}); ctx.beginPath(); ctx.arc(c.x, c.y, r, 0, PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.arc(c.x, c.y, r * 0.55, 0, PI * 2); ctx.stroke(); ctx.setLineDash({});
    if (tm < 0.6) {  // burst lines
      double k = tm / 0.6; ctx.beginPath();
      for (int i = 0; i < 14; i++) {
        double t = i / 14.0 * PI * 2 + 0.2, r0 = 20 + k * 90, r1 = r0 + 40 * (1 - k) + 12;
        ctx.moveTo(c.x + std::cos(t) * r0, c.y + std::sin(t) * r0); ctx.lineTo(c.x + std::cos(t) * r1, c.y + std::sin(t) * r1);
      }
      ctx.strokeStyle = centerA(1 - k); ctx.lineWidth = 1.4; ctx.stroke(); ctx.lineWidth = 1; ctx.strokeStyle = centerA(al);
    }
    ctx.beginPath(); ctx.moveTo(c.x - 12, c.y); ctx.lineTo(c.x + 12, c.y); ctx.moveTo(c.x, c.y - 12); ctx.lineTo(c.x, c.y + 12); ctx.stroke();
    double tx = c.x + 30, ty = c.y - 46;
    ctx.beginPath(); ctx.moveTo(c.x + 6, c.y - 6); ctx.lineTo(tx - 4, ty + 4); ctx.lineTo(tx + 150, ty + 4); ctx.stroke();
    ctx.fontSize = W < 600 ? 9 : 11; ctx.fontWeight = 500; ctx.textAlign = Align::Left; ctx.textBaseline = Baseline::Bottom;
    ctx.fillStyle = centerA(al); ctx.fillText("IMPACT · FRONT WHEEL INTO BOULDER", tx, ty);
    ctx.fillStyle = dimA(al); ctx.textBaseline = Baseline::Top;
    char buf[96]; std::snprintf(buf, sizeof buf, "vz %.1f m/s  ·  pitch %d°", crash.vz, crash.pitch);
    ctx.fillText(buf, tx, ty + 8);
  }
  ctx.restore();
}

/* ---------- parts list and title block ---------- */
void Saver::cell(double x, double y, double w, double h, const char* label, const std::string& value, double size, int weight) {
  ctx.strokeStyle = BORDER; ctx.lineWidth = 1; ctx.strokeRect(x + 0.5, y + 0.5, w, h);
  ctx.textAlign = Align::Left; ctx.textBaseline = Baseline::Top;
  bool hasLabel = label && *label;
  if (hasLabel) { ctx.fillStyle = DIM; ctx.fontSize = 8; ctx.fontWeight = 400; ctx.fillText(label, x + 6, y + 4); }
  if (!value.empty()) { ctx.fillStyle = LINE; ctx.fontSize = size; ctx.fontWeight = weight; ctx.fillText(value, x + 6, y + (hasLabel ? 14 : (h - size) / 2)); }
}
void Saver::projectionSymbol(double x, double y, double h) {  // third-angle projection
  ctx.strokeStyle = LINE; ctx.lineWidth = 1;
  double s = h * 0.32, cx = x, cy = y;
  ctx.beginPath(); ctx.moveTo(cx - s * 1.6, cy - s * 0.5); ctx.lineTo(cx - s * 0.2, cy - s); ctx.lineTo(cx - s * 0.2, cy + s); ctx.lineTo(cx - s * 1.6, cy + s * 0.5); ctx.closePath(); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx + s * 1.3, cy, s, 0, PI * 2); ctx.moveTo(cx + s * 1.3 + s * 0.5, cy); ctx.arc(cx + s * 1.3, cy, s * 0.5, 0, PI * 2); ctx.stroke();
  ctx.setLineDash({5, 2, 1, 2}); ctx.strokeStyle = CENTER; ctx.beginPath(); ctx.moveTo(cx - s * 2, cy); ctx.lineTo(cx + s * 2.6, cy); ctx.stroke(); ctx.setLineDash({});
}
static std::string Narrow(const wchar_t* w) {
  int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
  std::string s(n > 0 ? n - 1 : 0, '\0'); WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr); return s;
}
void Saver::drawTables(double a) {
  std::string scale = "1:" + std::to_string((long long)std::max(1.0, jsRound(1 / (S * 0.2646))));
  std::string view = state == RIDE ? (st > 4.3 ? "RIDE SIM · 0.55× SPEED" : "GENERAL ASSEMBLY") : state == CRASH ? "IMPACT · CRASH" : "REASSEMBLING";
  SYSTEMTIME t; GetLocalTime(&t);
  char today[16]; std::snprintf(today, sizeof today, "%04d-%02d-%02d", t.wYear, t.wMonth, t.wDay);
  wchar_t wclock[64] = L""; GetTimeFormatEx(LOCALE_NAME_USER_DEFAULT, TIME_NOSECONDS, &t, nullptr, wclock, 64);
  std::string clock = Narrow(wclock);
  ctx.fillStyle = GROUND;
  if (!wide) {
    double x0 = panel.x0, x1 = panel.x1, y0 = panel.y0, y1 = panel.y1, w = x1 - x0, hh = (y1 - y0) / 2;
    ctx.fillRect(x0, y0, w, y1 - y0);
    cell(x0, y0, w * 0.7, hh, "TITLE", "TRAIL BIKE 29 IN · FS-150", 10);
    cell(x0 + w * 0.7, y0, w * 0.3, hh, "SCALE", scale, 10);
    cell(x0, y0 + hh, w * 0.4, hh, "DWG NO", "MTB-29-150", 10);
    cell(x0 + w * 0.4, y0 + hh, w * 0.6, hh, "VIEW", view, 10);
    return;
  }
  double x0 = panel.x0, x1 = panel.x1, y0 = panel.y0, y1 = panel.y1, w = x1 - x0;
  double tbH = 164, ty = y1 - tbH;
  ctx.fillRect(x0, y0, w, y1 - y0);
  ctx.strokeStyle = BORDER; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x0, y1); ctx.stroke();
  // title block
  cell(x0, ty, w, 46, "TITLE", "FULL-SUSPENSION TRAIL BIKE, 29 IN", 12);
  ctx.fillStyle = THIN; ctx.fontSize = 9; ctx.fontWeight = 400; ctx.fillText("GENERAL ASSEMBLY & EXPLODED VIEW", x0 + 6, ty + 31);
  double r2 = ty + 46, rh = 39;
  cell(x0, r2, w * 0.46, rh, "DWG NO", "MTB-29-150");
  cell(x0 + w * 0.46, r2, w * 0.18, rh, "REV", "C");
  cell(x0 + w * 0.64, r2, w * 0.36, rh, "PROJECTION", "");
  projectionSymbol(x0 + w * 0.82, r2 + rh * 0.6, rh);
  double r3 = r2 + rh;
  cell(x0, r3, w * 0.3, rh, "SCALE", scale);
  cell(x0 + w * 0.3, r3, w * 0.22, rh, "UNITS", "MM");
  cell(x0 + w * 0.52, r3, w * 0.48, rh, "VIEW", view, 10);
  double r4 = r3 + rh;
  cell(x0, r4, w * 0.3, tbH - 46 - rh * 2, "SHEET", "1 / 1");
  cell(x0 + w * 0.3, r4, w * 0.4, tbH - 46 - rh * 2, "DATE", today, 10);
  cell(x0 + w * 0.7, r4, w * 0.3, tbH - 46 - rh * 2, "TIME", clock, 10);
  // parts list, item 1 at the bottom, header above
  double rowsAvail = ty - y0 - 12, rowH = std::min(20.0, std::floor(rowsAvail / (parts.size() + 1)));
  if (rowH < 12) return;
  double cw[4] = {30, (w - 64) * 0.5, (w - 64) * 0.5, 34}, cx[4] = {x0, x0 + cw[0], x0 + cw[0] + cw[1], x0 + cw[0] + cw[1] + cw[2]};
  double bottom = ty, fsz = rowH >= 17 ? 10 : 9;
  ctx.textBaseline = Baseline::Middle;
  auto rowAt = [&](size_t i) { return bottom - (i + 1) * rowH; };
  (void)a;
  for (size_t i = 0; i < parts.size(); i++) {
    const Part& p = parts[i]; double y = rowAt(i);
    for (int k = 0; k < 4; k++) { ctx.strokeStyle = BORDER; ctx.lineWidth = 1; ctx.strokeRect(cx[k] + 0.5, y + 0.5, cw[k], rowH); }
    ctx.fontSize = fsz; ctx.fontWeight = 500; ctx.fillStyle = DIM; ctx.textAlign = Align::Center; ctx.fillText(std::to_string(p.item), cx[0] + cw[0] / 2, y + rowH / 2 + 0.5);
    ctx.textAlign = Align::Left; ctx.fillStyle = LINE; ctx.fillText(upper(p.name), cx[1] + 6, y + rowH / 2 + 0.5);
    ctx.fontWeight = 400; ctx.fillStyle = THIN; ctx.fillText(p.spec, cx[2] + 6, y + rowH / 2 + 0.5);
    ctx.textAlign = Align::Center; ctx.fillStyle = LINE; ctx.fillText("1", cx[3] + cw[3] / 2, y + rowH / 2 + 0.5);
  }
  double hy = rowAt(parts.size());
  const char* heads[4] = {"ITEM", "DESCRIPTION", "SPECIFICATION", "QTY"};
  for (int k = 0; k < 4; k++) {
    ctx.strokeStyle = BORDER; ctx.strokeRect(cx[k] + 0.5, hy + 0.5, cw[k], rowH);
    ctx.fontSize = 8; ctx.fontWeight = 500; ctx.fillStyle = DIM; bool mid = k == 0 || k == 3; ctx.textAlign = mid ? Align::Center : Align::Left;
    ctx.fillText(heads[k], mid ? cx[k] + cw[k] / 2 : cx[k] + 6, hy + rowH / 2 + 0.5);
  }
  ctx.fontSize = 9; ctx.fontWeight = 500; ctx.fillStyle = DIM; ctx.textAlign = Align::Left; ctx.textBaseline = Baseline::Bottom;
  ctx.fillText("PARTS LIST", x0 + 6, hy - 4);
  // notes at the top of the panel
  double ny = y0 + 18;
  if (hy - ny > 70) {
    ctx.textBaseline = Baseline::Top; ctx.fillStyle = DIM; ctx.fontSize = 9; ctx.fontWeight = 500; ctx.fillText("NOTES", x0 + 6, ny);
    ctx.fillStyle = THIN; ctx.fontWeight = 400;
    const char* notes[3] = {"1. ALL DIMENSIONS IN MM UNLESS NOTED.", "2. GEOMETRY SHOWN AT 30% SAG.", "3. TORQUE AXLES TO SPEC BEFORE RIDING."};
    for (int i = 0; i < 3; i++) ctx.fillText(notes[i], x0 + 6, ny + 16 + i * 14);
  }
}

/* ---------- loop ---------- */
void Saver::frame(double dtIn) {
  double dt = std::min(0.05, dtIn);
  time += dt; st += dt;
  if (state == CRASH && st >= DUR_CRASH) setState(REASSEMBLE);
  else if (state == REASSEMBLE && st >= DUR_REASSEMBLE) setState(RIDE);
  if (state == RIDE) {
    const int n = 10; double h = dt * SLOWMO / n, cliff = ride.trail->start + ride.trail->len;
    bool landed = false; double lvz = 0; int lpitch = 0;
    for (int i = 0; i < n; i++) {
      double vz = ride.vz;
      rideStep(h);
      double fx = ride.X + 648 * std::cos(ride.th); Seg* T = ride.trail;
      if (!T->bxSet && fx > cliff) {  // takeoff: put the boulder where the front wheel is going to come down
        T->bx = clampd(predictTouchdown(cliff, T) + BX_OFFSET, FACE + 300, 9000); T->bxSet = true;
      }
      // crash: the front wheel's first touchdown after going off the drop (it lands nose-first on the boulder)
      if (fx > cliff + 520 && ride.sf > 0) { landed = true; lvz = vz / 1000; lpitch = (int)jsRound(ride.th * 180 / PI); break; }
    }
    double lead = std::min(0.0, ride.vz) * 0.3;  // look ahead when falling
    ride.camZ += (ride.z + lead - ride.camZ) * std::min(1.0, dt * 4);
    ride.camZ = clampd(ride.camZ, ride.z - 700, ride.z + 250);  // never let the bike leave the viewport
    camX = ride.X; camZ = ride.camZ;
    if (landed) {
      P = pose();
      crash = {bikeWorld({P.front.x + R_TIRE * 0.7, P.front.y - R_TIRE * 0.3}), time, lvz, lpitch}; haveCrash = true;
      setState(CRASH);
    } else if (st > RIDE_TIMEOUT) {
      crash = {bikeWorld({P.front.x, P.front.y - R_TIRE}), time, 0, 0}; haveCrash = true; setState(CRASH);
    }
  } else if (state == CRASH) {
    double speed = st < 0.08 ? 0.1 : 1.0;  // a beat on the impact, then real time
    const int n = 10; double h = dt * speed / n;
    for (int i = 0; i < n; i++) { crashStep(h); debrisStep(h); }
    double x0 = 1e300, x1 = -1e300, my = 0;
    for (const Part& p : parts) { x0 = std::min(x0, p.b.x); x1 = std::max(x1, p.b.x); my += p.b.y / parts.size(); }
    double k = std::min(1.0, dt * 1.6); camX += ((x0 + x1) / 2 - camX) * k; camZ += (my + 150 - camZ) * k;  // frame the whole wreck
    double fit = clampd(A.w * 0.9 / ((x1 - x0 + 1100) * S0), 0.24, 0.85);  // zoom out far enough to see every part
    crashZoom += (fit - crashZoom) * std::min(1.0, dt * 2.5);
  } else {
    double e = ease(std::min(1.0, st / DUR_REASSEMBLE));
    camX = lerp(camFrom.x, ride.X, e); camZ = lerp(camFrom.y, ride.camZ, e);
  }
  // camera shake: a hard jolt at impact that dies away
  if (state == CRASH) { double m = 22 * std::exp(-st * 5) * (st < 1.2 ? 1 : 0); shake = {(rnd() - 0.5) * 2 * m, (rnd() - 0.5) * 2 * m}; }
  else shake = {0, 0};
  double a = assembly();
  P = pose();
  wheelRot -= dt * (state == RIDE ? ride.v * SLOWMO / R_TIRE : state == REASSEMBLE ? 1.2 * a : 0);
  crank -= dt * (state == RIDE ? 3.4 : 0);
  double introAl = state == RIDE ? smooth(0.3, 1.1, st) * (1 - smooth(3.3, 3.8, st)) : 0;  // labels + dims on the still bike, before the trail
  // on wide screens the intro is drawn smaller to leave room for the label columns, then zooms in for the ride
  double zi = wide ? clampd((A.w - 2 * 175) / ((FRONT.x + 840) * S0), 0.5, 0.75) : 1;
  zoom = state == REASSEMBLE ? lerp(crashZoom, zi, a)
       : state == RIDE ? lerp(zi, 1, smooth(3.6, 4.5, st)) * lerp(1, FALL_ZOOM, smooth(-1800, 200, ride.X - ride.trail->start - ride.trail->len))  // pull back for the drop
       : crashZoom;
  S = S0 * zoom;

  ctx.begin();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawSheet();
  drawParts(a);
  if (!tiny) {
    ctx.save(); ctx.beginPath(); ctx.rect(A.x, A.y, A.w, A.h); ctx.clip(); drawAssemblyCallouts(introAl); ctx.restore();
    ctx.save(); ctx.beginPath(); ctx.rect(A.x, A.y, A.w, A.h); ctx.clip(); drawDims(introAl); ctx.restore();
    drawTables(a);
  }
  ctx.end();
}
