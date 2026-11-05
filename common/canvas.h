// A small subset of the HTML canvas 2D API on top of Direct2D and DirectWrite, so the drawing code
// can follow the original page closely. Paths are flattened to polylines in device space as they are
// built (like canvas, the transform applies when a point is added); line widths and dashes scale with
// the transform in force when stroke() is called.
#pragma once
#include <d2d1.h>
#include <dwrite.h>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

struct V2 { double x = 0, y = 0; };
inline V2 operator+(V2 a, V2 b) { return {a.x + b.x, a.y + b.y}; }
inline V2 operator-(V2 a, V2 b) { return {a.x - b.x, a.y - b.y}; }
inline V2 operator*(V2 a, double k) { return {a.x * k, a.y * k}; }

struct Col { float r, g, b, a; };
inline Col rgba(int r, int g, int b, double a = 1) { return {r / 255.f, g / 255.f, b / 255.f, (float)a}; }

enum class Align { Left, Center, Right };
enum class Baseline { Alphabetic, Top, Middle, Bottom };

class Canvas {
 public:
  struct Mat { double a = 1, b = 0, c = 0, d = 1, e = 0, f = 0; };

  Col strokeStyle{1, 1, 1, 1}, fillStyle{0, 0, 0, 1};
  double lineWidth = 1, globalAlpha = 1, lineDashOffset = 0;
  bool roundCap = false, roundJoin = false;
  double fontSize = 10; int fontWeight = 400;
  Align textAlign = Align::Left; Baseline textBaseline = Baseline::Alphabetic;

  bool init(ID2D1Factory* f, IDWriteFactory* dw, HWND hwnd);
  bool resize(UINT w, UINT h);
  void begin(); bool end();  // end() returns false when the device was lost and the target has to be recreated

  void save(); void restore();
  void setTransform(double a, double b, double c, double d, double e, double f);
  void translate(double x, double y); void rotate(double t); void scale(double sx, double sy);
  void setLineDash(std::vector<double> d) { dash_ = std::move(d); }

  void beginPath();
  void moveTo(double x, double y); void lineTo(double x, double y);
  void moveTo(V2 p) { moveTo(p.x, p.y); } void lineTo(V2 p) { lineTo(p.x, p.y); }
  void quadraticCurveTo(double cx, double cy, double x, double y);
  void arc(double cx, double cy, double r, double a0, double a1, bool ccw = false);
  void rect(double x, double y, double w, double h);
  void roundRect(double x, double y, double w, double h, double r);
  void closePath();
  void fill(bool evenOdd = false); void stroke(); void clip();
  void fillRect(double x, double y, double w, double h); void strokeRect(double x, double y, double w, double h);

  void fillText(const std::string& utf8, double x, double y);
  double measureText(const std::string& utf8);

 private:
  struct State { Mat m; Col strokeStyle, fillStyle; double lineWidth, globalAlpha, lineDashOffset; bool roundCap, roundJoin;
                 double fontSize; int fontWeight; Align textAlign; Baseline textBaseline; std::vector<double> dash; int clips; };
  struct Clip { bool axis; ID2D1Geometry* geom; };
  V2 ap(double x, double y) const { return {m_.a * x + m_.c * y + m_.e, m_.b * x + m_.d * y + m_.f}; }
  double scaleOf() const;
  ID2D1PathGeometry* build(bool filled, bool evenOdd);
  void brush(Col c);
  IDWriteTextFormat* format();
  struct Text { IDWriteTextLayout* layout; double width, height, baseline; };
  const Text* text(const std::string& utf8);
  ID2D1StrokeStyle* strokeStyleFor(double w);

  ID2D1Factory* f_ = nullptr; IDWriteFactory* dw_ = nullptr; HWND hwnd_ = nullptr;
  ID2D1HwndRenderTarget* rt_ = nullptr; ID2D1SolidColorBrush* brush_ = nullptr;
  ID2D1StrokeStyle* solid_[4] = {};
  std::map<std::pair<int, int>, IDWriteTextFormat*> formats_;
  std::unordered_map<std::string, Text> texts_;          // the same labels are drawn every frame: lay each out once
  std::map<std::vector<int>, ID2D1StrokeStyle*> dashed_;  // dash patterns, keyed in stroke widths
  // a path that is exactly one full circle is drawn with the ellipse calls instead of a geometry
  bool circle_ = false; D2D1_ELLIPSE circ_{};
  std::wstring family_;
  Mat m_; std::vector<double> dash_; int clips_ = 0;
  std::vector<State> stack_; std::vector<Clip> clipStack_;
  std::vector<std::vector<D2D1_POINT_2F>> figs_; std::vector<bool> closed_;
};
