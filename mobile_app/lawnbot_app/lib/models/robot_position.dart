class RobotPosition {
  final double lat;
  final double lng;
  final double heading;
  final DateTime timestamp;

  const RobotPosition({
    required this.lat,
    required this.lng,
    required this.heading,
    required this.timestamp,
  });

  factory RobotPosition.fromJson(Map<String, dynamic> json) {
    return RobotPosition(
      lat: (json['lat'] as num).toDouble(),
      lng: (json['lng'] as num).toDouble(),
      heading: (json['hdg'] as num?)?.toDouble() ?? 0.0,
      timestamp: DateTime.now(),
    );
  }

  /// Whether this fix arrived at most [maxAge] before [now].
  bool isFresh(DateTime now, {Duration maxAge = const Duration(seconds: 3)}) =>
      now.difference(timestamp) <= maxAge;

  Map<String, dynamic> toJson() => {
        'lat': lat,
        'lng': lng,
        'hdg': heading,
      };
}
