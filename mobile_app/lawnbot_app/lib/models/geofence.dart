import 'package:latlong2/latlong.dart';
import '../utils/polygon_checks.dart';

/// Why a drawn polygon cannot be sent to the robot.
enum GeofenceProblem {
  /// Fewer than [Geofence.minPoints] vertices.
  tooFewPoints,

  /// Two edges cross or touch each other.
  selfIntersecting,

  /// Simple polygon with an inward corner. The robot only accepts convex ones.
  notConvex,
}

class Geofence {
  /// The robot rejects fences with fewer vertices.
  static const int minPoints = 4;

  final List<LatLng> points;

  const Geofence({required this.points});

  /// First reason the polygon cannot be sent, or null if it is valid
  /// (at least [minPoints] vertices, simple and convex).
  GeofenceProblem? get problem {
    if (points.length < minPoints) return GeofenceProblem.tooFewPoints;
    if (isSelfIntersecting(points)) return GeofenceProblem.selfIntersecting;
    if (!isConvex(points)) return GeofenceProblem.notConvex;
    return null;
  }

  bool get isValid => problem == null;

  Map<String, dynamic> toJson() => {
        'points': points
            .map((p) => {'lat': p.latitude, 'lng': p.longitude})
            .toList(),
      };
}
