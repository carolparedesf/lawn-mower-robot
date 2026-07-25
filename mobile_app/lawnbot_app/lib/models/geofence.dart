import 'package:latlong2/latlong.dart';

class Geofence {
  final List<LatLng> points;

  const Geofence({required this.points});

  bool get isValid => points.length >= 3;

  Map<String, dynamic> toJson() => {
        'points': points
            .map((p) => {'lat': p.latitude, 'lng': p.longitude})
            .toList(),
      };
}
