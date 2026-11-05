#include "canvas.h"
#include <algorithm>
#include <cmath>

static const double TAU = 6.283185307179586;

static std::wstring Widen(const std::string& s) {
  std::wstring w(MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), w.data(), (int)w.size());
  return w;
}

bool Canvas::init(ID2D1Factory* f, IDWriteFactory* dw, HWND hwnd) {
  f_ = f; dw_ = dw; hwnd_ = hwnd;
  // the page asks for IBM Plex Mono; use it when it is installed, else Consolas
  IDWriteFontCollection* fc = nullptr; family_ = L"Consolas";
  if (SUCCEEDED(dw_->GetSystemFontCollection(&fc, FALSE))) {
    UINT32 i; BOOL ok = FALSE;
    if (SUCCEEDED(fc->FindFamilyName(L"IBM Plex Mono", &i, &ok)) && ok) family_ = L"IBM Plex Mono";
    fc->Release();
  }
  for (int k = 0; k < 4; k++) {
    D2D1_CAP_STYLE cap = (k & 1) ? D2D1_CAP_STYLE_ROUND : D2D1_CAP_STYLE_FLAT;
    D2D1_STROKE_STYLE_PROPERTIES p{cap, cap, cap, (k & 2) ? D2D1_LINE_JOIN_ROUND : D2D1_LINE_JOIN_MITER, 10.f, D2D1_DASH_STYLE_SOLID, 0};
    f_->CreateStrokeStyle(&p, nullptr, 0, &solid_[k]);
  }
  RECT rc; GetClientRect(hwnd, &rc);
  return resize(rc.right, rc.bottom);
}

bool Canvas::resize(UINT w, UINT h) {
  if (rt_) { if (SUCCEEDED(rt_->Resize(D2D1_SIZE_U{w, h}))) return true; rt_->Release(); rt_ = nullptr; }
  if (brush_) { brush_->Release(); brush_ = nullptr; }
  D2D1_RENDER_TARGET_PROPERTIES rp{D2D1_RENDER_TARGET_TYPE_DEFAULT, {DXGI_FORMAT_UNKNOWN, D2D1_ALPHA_MODE_UNKNOWN}, 96.f, 96.f,
                                   D2D1_RENDER_TARGET_USAGE_NONE, D2D1_FEATURE_LEVEL_DEFAULT};
  // presented immediately: the main loop paces frames to the compositor once for every monitor
  D2D1_HWND_RENDER_TARGET_PROPERTIES hp{hwnd_, D2D1_SIZE_U{std::max(1u, w), std::max(1u, h)}, D2D1_PRESENT_OPTIONS_IMMEDIATELY};
  if (FAILED(f_->CreateHwndRenderTarget(&rp, &hp, &rt_))) return false;
  rt_->SetTextAntialiasMode(D2D1_TEXT_ANTIALIAS_MODE_GRAYSCALE);
  D2D1_COLOR_F c{1, 1, 1, 1};
  return SUCCEEDED(rt_->CreateSolidColorBrush(&c, nullptr, &brush_));
}

void Canvas::begin() {
  rt_->BeginDraw();
  D2D1_MATRIX_3X2_F id{1, 0, 0, 1, 0, 0}; rt_->SetTransform(&id);
  stack_.clear(); m_ = Mat{}; dash_.clear(); clips_ = 0; globalAlpha = 1;
}

bool Canvas::end() {
  while (!clipStack_.empty()) {
    Clip c = clipStack_.back(); clipStack_.pop_back();
    if (c.axis) rt_->PopAxisAlignedClip(); else { rt_->PopLayer(); c.geom->Release(); }
  }
  HRESULT hr = rt_->EndDraw(); lastEnd_ = hr;
  if (hr == (HRESULT)D2DERR_RECREATE_TARGET) {
    rt_->Release(); rt_ = nullptr; RECT rc; GetClientRect(hwnd_, &rc); resize(rc.right, rc.bottom);
    return false;
  }
  return true;
}

void Canvas::save() {
  stack_.push_back({m_, strokeStyle, fillStyle, lineWidth, globalAlpha, lineDashOffset, roundCap, roundJoin,
                    fontSize, fontWeight, textAlign, textBaseline, dash_, clips_});
  clips_ = 0;
}

void Canvas::restore() {
  if (stack_.empty()) return;
  for (; clips_ > 0; clips_--) {
    Clip c = clipStack_.back(); clipStack_.pop_back();
    if (c.axis) rt_->PopAxisAlignedClip(); else { rt_->PopLayer(); c.geom->Release(); }
  }
  State& s = stack_.back();
  m_ = s.m; strokeStyle = s.strokeStyle; fillStyle = s.fillStyle; lineWidth = s.lineWidth; globalAlpha = s.globalAlpha;
  lineDashOffset = s.lineDashOffset; roundCap = s.roundCap; roundJoin = s.roundJoin; fontSize = s.fontSize; fontWeight = s.fontWeight;
  textAlign = s.textAlign; textBaseline = s.textBaseline; dash_ = s.dash; clips_ = s.clips;
  stack_.pop_back();
}

void Canvas::setTransform(double a, double b, double c, double d, double e, double f) { m_ = {a, b, c, d, e, f}; }
void Canvas::translate(double x, double y) { m_.e += m_.a * x + m_.c * y; m_.f += m_.b * x + m_.d * y; }
void Canvas::rotate(double t) {
  double cs = std::cos(t), sn = std::sin(t), a = m_.a, b = m_.b, c = m_.c, d = m_.d;
  m_.a = a * cs + c * sn; m_.b = b * cs + d * sn; m_.c = -a * sn + c * cs; m_.d = -b * sn + d * cs;
}
void Canvas::scale(double sx, double sy) { m_.a *= sx; m_.b *= sx; m_.c *= sy; m_.d *= sy; }
double Canvas::scaleOf() const { return std::sqrt(std::fabs(m_.a * m_.d - m_.b * m_.c)); }

void Canvas::beginPath() { figs_.clear(); closed_.clear(); circle_ = false; }
void Canvas::moveTo(double x, double y) { circle_ = false; V2 p = ap(x, y); figs_.push_back({{(float)p.x, (float)p.y}}); closed_.push_back(false); }
void Canvas::lineTo(double x, double y) {
  if (figs_.empty()) { moveTo(x, y); return; }
  circle_ = false; V2 p = ap(x, y); figs_.back().push_back({(float)p.x, (float)p.y});
}
void Canvas::quadraticCurveTo(double cx, double cy, double x, double y) {
  if (figs_.empty()) moveTo(cx, cy);
  circle_ = false;
  D2D1_POINT_2F p0 = figs_.back().back(); V2 c = ap(cx, cy), p1 = ap(x, y);
  for (int i = 1; i <= 16; i++) {
    double t = i / 16.0, u = 1 - t;
    figs_.back().push_back({(float)(u * u * p0.x + 2 * u * t * c.x + t * t * p1.x), (float)(u * u * p0.y + 2 * u * t * c.y + t * t * p1.y)});
  }
}
void Canvas::arc(double cx, double cy, double r, double a0, double a1, bool ccw) {
  double sweep;
  if (std::fabs(a1 - a0) >= TAU - 1e-9) sweep = ccw ? -TAU : TAU;  // whole circle either way
  else if (!ccw) { sweep = std::fmod(a1 - a0, TAU); if (sweep < 0) sweep += TAU; }
  else { sweep = std::fmod(a0 - a1, TAU); if (sweep < 0) sweep += TAU; sweep = -sweep; }
  double rd = r * scaleOf();  // keep the chord error well under a pixel
  bool whole = figs_.empty() && std::fabs(sweep) >= TAU - 1e-9;
  V2 c = ap(cx, cy);
  int n = std::clamp((int)std::ceil(std::fabs(sweep) / std::sqrt(1.2 / std::max(rd, 0.5))), 4, 360);
  for (int i = 0; i <= n; i++) {
    double a = a0 + sweep * i / n, x = cx + std::cos(a) * r, y = cy + std::sin(a) * r;
    if (i == 0 && figs_.empty()) moveTo(x, y); else lineTo(x, y);
  }
  if (whole) { circle_ = true; circ_ = D2D1_ELLIPSE{{(float)c.x, (float)c.y}, (float)rd, (float)rd}; }
}
void Canvas::rect(double x, double y, double w, double h) { moveTo(x, y); lineTo(x + w, y); lineTo(x + w, y + h); lineTo(x, y + h); closePath(); }
void Canvas::roundRect(double x, double y, double w, double h, double r) {
  r = std::min({r, std::fabs(w) / 2, std::fabs(h) / 2});
  moveTo(x + r, y);
  bool hadFig = true; (void)hadFig;
  auto corner = [&](double ccx, double ccy, double s) {
    for (int i = 0; i <= 6; i++) { double a = s + (TAU / 4) * i / 6; lineTo(ccx + std::cos(a) * r, ccy + std::sin(a) * r); }
  };
  corner(x + w - r, y + r, -TAU / 4); corner(x + w - r, y + h - r, 0); corner(x + r, y + h - r, TAU / 4); corner(x + r, y + r, TAU / 2);
  closePath();
}
void Canvas::closePath() {
  if (figs_.empty()) return;
  circle_ = false;
  closed_.back() = true;
  D2D1_POINT_2F s = figs_.back().front();
  figs_.push_back({s}); closed_.push_back(false);  // like canvas: a new subpath starts where the closed one began
}

ID2D1PathGeometry* Canvas::build(bool filled, bool evenOdd) {
  ID2D1PathGeometry* g = nullptr; ID2D1GeometrySink* sk = nullptr;
  if (FAILED(f_->CreatePathGeometry(&g))) return nullptr;
  if (FAILED(g->Open(&sk))) { g->Release(); return nullptr; }
  sk->SetFillMode(evenOdd ? D2D1_FILL_MODE_ALTERNATE : D2D1_FILL_MODE_WINDING);
  for (size_t i = 0; i < figs_.size(); i++) {
    const auto& f = figs_[i];
    if (f.size() < 2) continue;
    sk->BeginFigure(f[0], filled ? D2D1_FIGURE_BEGIN_FILLED : D2D1_FIGURE_BEGIN_HOLLOW);
    sk->AddLines(f.data() + 1, (UINT32)(f.size() - 1));
    sk->EndFigure(closed_[i] ? D2D1_FIGURE_END_CLOSED : D2D1_FIGURE_END_OPEN);
  }
  sk->Close(); sk->Release();
  return g;
}

void Canvas::brush(Col c) { D2D1_COLOR_F k{c.r, c.g, c.b, (float)(c.a * globalAlpha)}; brush_->SetColor(&k); }

void Canvas::fill(bool evenOdd) {
  if (circle_) { brush(fillStyle); rt_->FillEllipse(&circ_, brush_); return; }
  ID2D1PathGeometry* g = build(true, evenOdd); if (!g) return;
  brush(fillStyle); rt_->FillGeometry(g, brush_, nullptr); g->Release();
}

ID2D1StrokeStyle* Canvas::strokeStyleFor(double w) {
  if (dash_.empty()) return solid_[(roundCap ? 1 : 0) | (roundJoin ? 2 : 0)];
  // Direct2D measures dashes in stroke widths; canvas in user units
  double k = scaleOf();
  std::vector<float> d; for (double v : dash_) d.push_back((float)std::max(1e-3, v * k / w));
  if (d.size() % 2) { auto e = d; d.insert(d.end(), e.begin(), e.end()); }
  float off = (float)(lineDashOffset * k / w);
  std::vector<int> key{roundCap, roundJoin, (int)std::lround(off * 1000)};
  for (float v : d) key.push_back((int)std::lround(v * 1000));
  auto it = dashed_.find(key);
  if (it != dashed_.end()) return it->second;
  if (dashed_.size() > 256) { for (auto& e : dashed_) e.second->Release(); dashed_.clear(); }  // the chain's moving offset keeps adding styles
  D2D1_CAP_STYLE cap = roundCap ? D2D1_CAP_STYLE_ROUND : D2D1_CAP_STYLE_FLAT;
  D2D1_STROKE_STYLE_PROPERTIES p{cap, cap, cap, roundJoin ? D2D1_LINE_JOIN_ROUND : D2D1_LINE_JOIN_MITER, 10.f, D2D1_DASH_STYLE_CUSTOM, off};
  ID2D1StrokeStyle* st = nullptr;
  if (FAILED(f_->CreateStrokeStyle(&p, d.data(), (UINT32)d.size(), &st))) return solid_[0];
  dashed_[key] = st;
  return st;
}

ID2D1GradientStopCollection* Canvas::gradient(const std::vector<Stop>& stops) {
  std::vector<D2D1_GRADIENT_STOP> g;
  for (const Stop& st : stops) g.push_back({st.pos, {st.col.r, st.col.g, st.col.b, (float)(st.col.a * globalAlpha)}});
  ID2D1GradientStopCollection* c = nullptr;
  rt_->CreateGradientStopCollection(g.data(), (UINT32)g.size(), D2D1_GAMMA_2_2, D2D1_EXTEND_MODE_CLAMP, &c);
  return c;
}

void Canvas::fillLinear(V2 p0, V2 p1, const std::vector<Stop>& stops) {
  ID2D1PathGeometry* g = build(true, false); if (!g) return;
  ID2D1GradientStopCollection* sc = gradient(stops);
  V2 a = ap(p0.x, p0.y), b = ap(p1.x, p1.y);
  D2D1_LINEAR_GRADIENT_BRUSH_PROPERTIES lp{{(float)a.x, (float)a.y}, {(float)b.x, (float)b.y}};
  ID2D1LinearGradientBrush* br = nullptr;
  if (sc && SUCCEEDED(rt_->CreateLinearGradientBrush(&lp, nullptr, sc, &br))) { rt_->FillGeometry(g, br, nullptr); br->Release(); }
  if (sc) sc->Release();
  g->Release();
}

void Canvas::fillRadial(V2 c, double rx, double ry, const std::vector<Stop>& stops) {
  ID2D1PathGeometry* g = build(true, false); if (!g) return;
  ID2D1GradientStopCollection* sc = gradient(stops);
  V2 a = ap(c.x, c.y); double k = scaleOf();
  D2D1_RADIAL_GRADIENT_BRUSH_PROPERTIES rp{{(float)a.x, (float)a.y}, {0, 0}, (float)(rx * k), (float)(ry * k)};
  ID2D1RadialGradientBrush* br = nullptr;
  if (sc && SUCCEEDED(rt_->CreateRadialGradientBrush(&rp, nullptr, sc, &br))) { rt_->FillGeometry(g, br, nullptr); br->Release(); }
  if (sc) sc->Release();
  g->Release();
}

void Canvas::stroke() {
  double w = std::max(1e-3, lineWidth * scaleOf());
  ID2D1StrokeStyle* style = strokeStyleFor(w);
  brush(strokeStyle);
  if (circle_) { rt_->DrawEllipse(&circ_, brush_, (float)w, style); return; }
  bool lines = true;  // only straight two-point segments: draw them directly, no geometry
  for (size_t i = 0; i < figs_.size() && lines; i++) lines = figs_[i].size() < 2 || (figs_[i].size() == 2 && !closed_[i]);
  if (lines) {
    for (const auto& f : figs_) if (f.size() == 2) rt_->DrawLine(f[0], f[1], brush_, (float)w, style);
    return;
  }
  ID2D1PathGeometry* g = build(false, false); if (!g) return;
  rt_->DrawGeometry(g, brush_, (float)w, style);
  g->Release();
}

void Canvas::clip() {
  // one closed axis-aligned rectangle: a cheap scissor clip; anything else: a geometry layer
  if (figs_.size() >= 1 && figs_[0].size() == 4 && closed_[0] && (figs_.size() == 1 || figs_[1].size() < 2)) {
    const auto& q = figs_[0];
    bool axis = (q[0].y == q[1].y && q[1].x == q[2].x && q[2].y == q[3].y && q[3].x == q[0].x) ||
                (q[0].x == q[1].x && q[1].y == q[2].y && q[2].x == q[3].x && q[3].y == q[0].y);
    if (axis) {
      D2D1_RECT_F r{std::min({q[0].x, q[1].x, q[2].x}), std::min({q[0].y, q[1].y, q[2].y}),
                    std::max({q[0].x, q[1].x, q[2].x}), std::max({q[0].y, q[1].y, q[2].y})};
      rt_->PushAxisAlignedClip(&r, D2D1_ANTIALIAS_MODE_ALIASED);
      clipStack_.push_back({true, nullptr}); clips_++;
      return;
    }
  }
  ID2D1PathGeometry* g = build(true, false); if (!g) return;
  D2D1_LAYER_PARAMETERS lp{};
  lp.contentBounds = D2D1_RECT_F{-1e7f, -1e7f, 1e7f, 1e7f}; lp.geometricMask = g;
  lp.maskAntialiasMode = D2D1_ANTIALIAS_MODE_PER_PRIMITIVE; lp.maskTransform = D2D1_MATRIX_3X2_F{1, 0, 0, 1, 0, 0};
  lp.opacity = 1; lp.layerOptions = D2D1_LAYER_OPTIONS_NONE;
  rt_->PushLayer(&lp, nullptr);
  clipStack_.push_back({false, g}); clips_++;
}

void Canvas::fillRect(double x, double y, double w, double h) {
  auto f = figs_; auto c = closed_; beginPath(); rect(x, y, w, h); fill(); figs_ = f; closed_ = c;
}
void Canvas::strokeRect(double x, double y, double w, double h) {
  auto f = figs_; auto c = closed_; beginPath(); rect(x, y, w, h); stroke(); figs_ = f; closed_ = c;
}

IDWriteTextFormat* Canvas::format() {
  auto key = std::make_pair((int)std::lround(fontSize * 10), fontWeight);
  auto it = formats_.find(key);
  if (it != formats_.end()) return it->second;
  IDWriteTextFormat* fmt = nullptr;
  dw_->CreateTextFormat(family_.c_str(), nullptr, (DWRITE_FONT_WEIGHT)fontWeight, DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL,
                        (float)fontSize, L"en-us", &fmt);
  if (fmt) fmt->SetWordWrapping(DWRITE_WORD_WRAPPING_NO_WRAP);
  formats_[key] = fmt;
  return fmt;
}

const Canvas::Text* Canvas::text(const std::string& utf8) {
  std::string key = utf8; key += '\x01'; key += std::to_string((int)std::lround(fontSize * 10)); key += '/'; key += std::to_string(fontWeight);
  auto it = texts_.find(key);
  if (it != texts_.end()) return &it->second;
  if (texts_.size() > 3000) { for (auto& e : texts_) e.second.layout->Release(); texts_.clear(); }  // the clock and readouts change
  IDWriteTextFormat* fmt = format(); if (!fmt) return nullptr;
  std::wstring w = Widen(utf8); IDWriteTextLayout* l = nullptr;
  if (FAILED(dw_->CreateTextLayout(w.c_str(), (UINT32)w.size(), fmt, 100000.f, 1000.f, &l)) || !l) return nullptr;
  DWRITE_TEXT_METRICS tm{}; l->GetMetrics(&tm);
  DWRITE_LINE_METRICS lm{}; UINT32 n = 0; l->GetLineMetrics(&lm, 1, &n);
  return &(texts_[key] = Text{l, tm.widthIncludingTrailingWhitespace, lm.height, lm.baseline});
}

double Canvas::measureText(const std::string& utf8) { const Text* t = text(utf8); return t ? t->width : 0; }

void Canvas::fillText(const std::string& utf8, double x, double y) {
  const Text* t = text(utf8); if (!t) return;
  double x0 = textAlign == Align::Center ? x - t->width / 2 : textAlign == Align::Right ? x - t->width : x;
  double y0 = textBaseline == Baseline::Top ? y : textBaseline == Baseline::Middle ? y - t->height / 2 : textBaseline == Baseline::Bottom ? y - t->height : y - t->baseline;
  D2D1_MATRIX_3X2_F m{(float)m_.a, (float)m_.b, (float)m_.c, (float)m_.d, (float)m_.e, (float)m_.f}, id{1, 0, 0, 1, 0, 0};
  rt_->SetTransform(&m);
  brush(fillStyle);
  rt_->DrawTextLayout(D2D1_POINT_2F{(float)x0, (float)y0}, t->layout, brush_, D2D1_DRAW_TEXT_OPTIONS_NONE);
  rt_->SetTransform(&id);
}
