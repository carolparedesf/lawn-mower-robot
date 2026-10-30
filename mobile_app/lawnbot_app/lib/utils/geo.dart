import 'dart:math' as math;
import 'package:latlong2/latlong.dart';

/// Local tangent plane (x = east, y = north, meters) around a datum, the
/// inverse of the robot's `lat_lon_to_local_xy` (geofence_logic.py): same
/// meters-per-degree formulas, so both sides agree.
LatLng localToLatLng(double x, double y, double lat0, double lng0) {
  final phi = lat0 * math.pi / 180.0;
  final mPerDegLat = 111132.92 -
      559.82 * math.cos(2 * phi) +
      1.175 * math.cos(4 * phi) -
      0.0023 * math.cos(6 * phi);
  final mPerDegLng = 111412.84 * math.cos(phi) -
      93.5 * math.cos(3 * phi) +
      0.118 * math.cos(5 * phi);
  return LatLng(lat0 + y / mPerDegLat, lng0 + x / mPerDegLng);
}
