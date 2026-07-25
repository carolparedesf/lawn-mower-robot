class RobotStatus {
  final String mode;
  final int battery;
  final String alert;
  final bool blade;
  final bool trimmer;
  final bool charging;

  const RobotStatus({
    required this.mode,
    required this.battery,
    required this.alert,
    this.blade = false,
    this.trimmer = false,
    this.charging = false,
  });

  factory RobotStatus.fromJson(Map<String, dynamic> json) {
    return RobotStatus(
      mode: json['mode'] as String? ?? 'idle',
      battery: (json['battery'] as num?)?.toInt() ?? 0,
      alert: json['alert'] as String? ?? 'none',
      blade: json['blade'] as bool? ?? false,
      trimmer: json['trimmer'] as bool? ?? false,
      charging: json['charging'] as bool? ?? false,
    );
  }

  Map<String, dynamic> toJson() => {
        'mode': mode,
        'battery': battery,
        'alert': alert,
        'blade': blade,
        'trimmer': trimmer,
        'charging': charging,
      };

  bool get hasGeofenceBreach => alert == 'geofence_breach';

  bool get isAutonomous => mode.startsWith('auto_');
}
