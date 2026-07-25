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

  Map<String, dynamic> toJson() => {
        'lat': lat,
        'lng': lng,
        'hdg': heading,
      };
}
