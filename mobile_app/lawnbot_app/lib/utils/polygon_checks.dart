import 'dart:math' as math;
import 'package:latlong2/latlong.dart';

/// Planar checks for polygons drawn on the map. Coordinates are projected to
/// meters (x east, y north) around the first point, which is accurate enough
/// for a lawn-sized area.

List<math.Point<double>> _project(List<LatLng> points) {
  final origin = points.first;
  const metersPerDegLat = 110540.0;
  final metersPerDegLng =
      111320.0 * math.cos(origin.latitude * math.pi / 180.0);
  return [
    for (final p in points)
      math.Point<double>(
        (p.longitude - origin.longitude) * metersPerDegLng,
        (p.latitude - origin.latitude) * metersPerDegLat,
      ),
  ];
}

double _cross(math.Point<double> o, math.Point<double> a, math.Point<double> b) =>
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);

bool _withinBox(
    math.Point<double> p, math.Point<double> q, math.Point<double> r) {
  const eps = 1e-9;
  return r.x >= math.min(p.x, q.x) - eps &&
      r.x <= math.max(p.x, q.x) + eps &&
      r.y >= math.min(p.y, q.y) - eps &&
      r.y <= math.max(p.y, q.y) + eps;
}

/// True if segments p1-p2 and p3-p4 cross, touch at an end or overlap.
bool _segmentsIntersect(
  math.Point<double> p1,
  math.Point<double> p2,
  math.Point<double> p3,
  math.Point<double> p4,
) {
  const eps = 1e-9;
  final d1 = _cross(p3, p4, p1);
  final d2 = _cross(p3, p4, p2);
  final d3 = _cross(p1, p2, p3);
  final d4 = _cross(p1, p2, p4);
  if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
      ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) {
    return true;
  }
  if (d1.abs() <= eps && _withinBox(p3, p4, p1)) return true;
  if (d2.abs() <= eps && _withinBox(p3, p4, p2)) return true;
  if (d3.abs() <= eps && _withinBox(p1, p2, p3)) return true;
  if (d4.abs() <= eps && _withinBox(p1, p2, p4)) return true;
  return false;
}

/// True if any two edges of the closed polygon cross or touch each other
/// (adjacent edges may only share their common vertex).
bool isSelfIntersecting(List<LatLng> points) {
  if (points.length < 4) return false;
  final pts = _project(points);
  final n = pts.length;

  for (var i = 0; i < n; i++) {
    final a = pts[i];
    final b = pts[(i + 1) % n];
    final c = pts[(i + 2) % n];
    // Adjacent edges folding back over each other.
    final backtracks = _cross(a, b, c).abs() <= 1e-9 &&
        ((b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y)) < 0;
    if (backtracks) return true;

    for (var j = i + 2; j < n; j++) {
      if (i == 0 && j == n - 1) continue;
      if (_segmentsIntersect(a, b, pts[j], pts[(j + 1) % n])) return true;
    }
  }
  return false;
}

/// True if the closed polygon is convex: every turn goes to the same side.
/// Collinear vertices are allowed. Meant for simple polygons: check
/// [isSelfIntersecting] first.
bool isConvex(List<LatLng> points) {
  if (points.length < 3) return false;
  final pts = _project(points);
  final n = pts.length;
  var sign = 0;

  for (var i = 0; i < n; i++) {
    final a = pts[i];
    final b = pts[(i + 1) % n];
    final c = pts[(i + 2) % n];
    final e1 = math.sqrt(math.pow(b.x - a.x, 2) + math.pow(b.y - a.y, 2));
    final e2 = math.sqrt(math.pow(c.x - b.x, 2) + math.pow(c.y - b.y, 2));
    if (e1 < 1e-9 || e2 < 1e-9) continue; // repeated point

    final cross = _cross(a, b, c);
    if (cross.abs() <= 1e-3 * e1 * e2) continue; // collinear (< ~0.06 deg)
    final s = cross > 0 ? 1 : -1;
    if (sign == 0) {
      sign = s;
    } else if (s != sign) {
      return false;
    }
  }
  return sign != 0;
}
