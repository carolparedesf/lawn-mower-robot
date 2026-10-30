class RobotStatus {
  final String mode;
  /// Battery percentage, or null when the robot does not report a valid one
  /// (the bridge omits the key if the battery sensor is not connected).
  final int? battery;
  final String alert;
  final bool blade;
  final bool trimmer;
  final bool charging;

  /// Status of the fence reported by the robot (/geofence/status): "OK",
  /// "INVALID:<reason>", "WAITING_CONVERSION", "NO_POSE" or "NO_POLYGON".
  /// Null if the robot does not report it.
  final String? geofenceStatus;

  const RobotStatus({
    required this.mode,
    required this.battery,
    required this.alert,
    this.blade = false,
    this.trimmer = false,
    this.charging = false,
    this.geofenceStatus,
  });

  factory RobotStatus.fromJson(Map<String, dynamic> json) {
    return RobotStatus(
      mode: json['mode'] as String? ?? 'idle',
      battery: (json['battery'] as num?)?.toInt(),
      alert: json['alert'] as String? ?? 'none',
      blade: json['blade'] as bool? ?? false,
      trimmer: json['trimmer'] as bool? ?? false,
      charging: json['charging'] as bool? ?? false,
      geofenceStatus: json['geofence'] as String?,
    );
  }

  Map<String, dynamic> toJson() => {
        'mode': mode,
        if (battery != null) 'battery': battery,
        'alert': alert,
        'blade': blade,
        'trimmer': trimmer,
        'charging': charging,
        if (geofenceStatus != null) 'geofence': geofenceStatus,
      };

  bool get hasGeofenceBreach => alert == 'geofence_breach';

  bool get isAutonomous => mode.startsWith('auto_');
}
