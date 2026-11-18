import 'dart:math' as math;
import 'package:latlong2/latlong.dart';

/// Meters per degree of latitude / longitude at [latDeg] (same formulas as the
/// robot's `geofence_logic.py`).
({double lat, double lng}) _metersPerDeg(double latDeg) {
  final phi = latDeg * math.pi / 180.0;
  return (
    lat: 111132.92 -
        559.82 * math.cos(2 * phi) +
        1.175 * math.cos(4 * phi) -
        0.0023 * math.cos(6 * phi),
    lng: 111412.84 * math.cos(phi) -
        93.5 * math.cos(3 * phi) +
        0.118 * math.cos(5 * phi),
  );
}

/// Local tangent plane (x = east, y = north, meters) around a datum, the
/// inverse of the robot's `lat_lon_to_local_xy` (geofence_logic.py): same
/// meters-per-degree formulas, so both sides agree.
LatLng localToLatLng(double x, double y, double lat0, double lng0) {
  final m = _metersPerDeg(lat0);
  return LatLng(lat0 + y / m.lat, lng0 + x / m.lng);
}

/// Inverse of [localToLatLng]: (east, north) meters of [p] from the datum.
({double x, double y}) latLngToLocal(LatLng p, double lat0, double lng0) {
  final m = _metersPerDeg(lat0);
  return (
    x: (p.longitude - lng0) * m.lng,
    y: (p.latitude - lat0) * m.lat,
  );
}

/// Size of a drawn polygon in meters.
class PolygonSize {
  /// East-west extent of the bounding box.
  final double eastWest;

  /// North-south extent of the bounding box.
  final double northSouth;

  /// Enclosed area in m² (0 with fewer than 3 points).
  final double area;

  const PolygonSize(this.eastWest, this.northSouth, this.area);
}

/// Measures [points] on the local plane around the first one, or null if empty.
PolygonSize? polygonSize(List<LatLng> points) {
  if (points.isEmpty) return null;
  final lat0 = points.first.latitude, lng0 = points.first.longitude;
  final xy = [for (final p in points) latLngToLocal(p, lat0, lng0)];
  final xs = xy.map((p) => p.x), ys = xy.map((p) => p.y);
  var twice = 0.0;
  if (xy.length >= 3) {
    for (var i = 0; i < xy.length; i++) {
      final a = xy[i], b = xy[(i + 1) % xy.length];
      twice += a.x * b.y - b.x * a.y;
    }
  }
  return PolygonSize(
    xs.reduce(math.max) - xs.reduce(math.min),
    ys.reduce(math.max) - ys.reduce(math.min),
    twice.abs() / 2,
  );
}
