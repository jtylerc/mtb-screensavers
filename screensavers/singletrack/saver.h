// One running copy of the Singletrack animation: its own trail, ride and crash, drawn into one window.
#pragma once
#include <deque>
#include <random>
#include <string>
#include <vector>
#include "canvas.h"
#include "scene.h"

class Saver : public Scene {
 public:
  bool init(ID2D1Factory* f, IDWriteFactory* dw, HWND hwnd, unsigned seed) override;
  void resized() override;
  void frame(double dt) override;  // advance and draw one frame

 private:
  // ---- trail ----
  struct Seg { double s, start, len, drop, seed, y0 = 0, bx = 0; bool bxSet = false; };
  struct Ride { double X = 0, v = 0, z = 0, vz = 0, th = 0, vth = 0, sf = 0, sr = 0, camZ = 0, cf = 0, cr = 0;
                Seg* trail = nullptr; std::deque<Seg> segs; };
  struct Pose { double tf, tr, phi; V2 front, rear, rock0, rock2; };
  struct Body { double x, y, a, I, vx, vy, w; std::vector<V2> hull; };
  struct Part { int id; V2 home; int hp; double r; const char* name; const char* spec; bool local; int item;
                Body b; double fx = 0, fy = 0, fa = 0, target = 0; };
  struct Debris { double x, y, vx, vy, a, w, r, life = 0; int n; bool shard; };
  struct Crash { V2 w; double t0, vz; int pitch; };

  double rnd() { return std::uniform_real_distribution<double>(0, 1)(rng_); }
  void doResize();
  // terrain
  double hsh(double n) const; double lin(double u, double cell, double k) const; double rough(double u) const;
  double rockH(double u, const Seg* T); double trailH(double u, const Seg* T);
  Seg* segAt(double x); double groundH(double x);
  double belowCliff(double u, double v, double bottom, const Seg* T); double runout(const Seg* T) const;
  double boulderTop(double v, const Seg* T) const; double contactY(double x);
  // ride
  void rideReset(double baseY); void rideStep(double h); double predictTouchdown(double cliff, Seg* T); Pose pose() const;
  // states
  void setState(int s);
  std::vector<Debris> burst(double x, double y, int n, bool rock);
  void debrisStep(double h); void crashStep(double h);
  // mapping
  V2 homeScreen(V2 h) const; V2 ws(double wx, double wy) const; V2 bikeWorld(V2 p) const; V2 bikeToScreen(V2 p) const;
  V2 partPivot(const Part& p) const;
  // drawing helpers
  double px(double n) const { return n * U_; }
  void seg(V2 a, V2 b, double w, Col c); void ring(V2 c, double r, double w, Col col);
  template <class F> void shape(F build, double edge = 1.3);
  void circ(V2 c, double r, double edge = 1.3);
  struct Tube { V2 a, b; double w; };
  void tubes(const std::vector<Tube>& list, bool round = true);
  template <class F> void strokeOutlined(F build, double w);
  void centerMark(double r);
  // the parts
  void drawPart(int id); void drawWheel(bool front); void drawRotor(double R); void drawCassette(); void drawCrankset();
  void drawDerailleur(); void drawFrame(); void drawSwingarm(); void drawShock(); void drawFork(); void drawCockpit(); void drawDropper();
  void drawChain(double alpha);
  // the sheet
  void drawSheet(); void drawTerrain(double al); void drawDebris(); void drawParts(double a);
  void balloon(V2 p0, V2 bc, double br, int num, const std::string& name, double al, double fs);
  void drawAssemblyCallouts(double al);
  void arrowHead(V2 p, V2 dir); void dimText(const std::string& t, double x, double y, double al);
  void hDim(V2 P1, V2 P2, double y, const std::string& text, double al);
  void angDim(V2 V, double r, double a0, double a1, const std::string& text, double al);
  void drawDims(double al);
  void cell(double x, double y, double w, double h, const char* label, const std::string& value, double size = 11, int weight = 500);
  void projectionSymbol(double x, double y, double h);
  void drawTables(double a);
  double assembly() const;

  Canvas ctx; HWND hwnd_ = nullptr; std::mt19937 rng_;
  double W = 0, H = 0, dpr = 1, S = 1, S0 = 1, zoom = 1, U_ = 1;
  struct { double x, y, w, h, cx, cy; } A{};
  struct { double x0, x1, y0, y1; } panel{};
  double mo = 12, mi = 30; bool wide = false, tiny = false;  // tiny: the preview monitor in Screen Saver Settings
  mutable double trailSeed = 1;
  Ride ride; Pose P{};
  double camX = 0, camZ = 0, crashZoom = 1; V2 camFrom{}; V2 shake{};
  std::vector<Debris> debris; std::vector<Part> parts;
  int state = 0; double st = 0, time = 0, wheelRot = 0, crank = 0;
  Crash crash{}; bool haveCrash = false;
};
