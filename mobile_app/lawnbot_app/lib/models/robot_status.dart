class RobotStatus {
  final String mode;
  /// Battery percentage, or null when the robot does not report a valid one
  /// (the bridge omits the key if the battery sensor is not connected).
  final int? battery;
  final String alert;
  final bool blade;
  final bool trimmer;
  final bool charging;

  /// "Forced cut" request accepted by the robot: overrides ONLY the grass
  /// detection condition of the blade. Optional key `force_cut`.
  final bool forceCut;

  /// What the motors are really doing (`blade`/`trimmer` above are what was
  /// requested and accepted; a requested motor may be off because a safety
  /// condition is not met). Optional keys `blade_running`/`trimmer_running`.
  final bool bladeRunning;
  final bool trimmerRunning;

  /// Text of the robot's /blade/status explaining why each motor runs or not
  /// (see parseBladeStatus). Optional key `blade_status`.
  final String? bladeStatus;

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
    this.forceCut = false,
    this.bladeRunning = false,
    this.trimmerRunning = false,
    this.bladeStatus,
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
      forceCut: json['force_cut'] as bool? ?? false,
      bladeRunning: json['blade_running'] as bool? ?? false,
      trimmerRunning: json['trimmer_running'] as bool? ?? false,
      bladeStatus: json['blade_status'] as String?,
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
        'force_cut': forceCut,
        'blade_running': bladeRunning,
        'trimmer_running': trimmerRunning,
        if (bladeStatus != null) 'blade_status': bladeStatus,
        if (geofenceStatus != null) 'geofence': geofenceStatus,
      };

  bool get hasGeofenceBreach => alert == 'geofence_breach';

  bool get isAutonomous => mode.startsWith('auto_');

  /// The robot attends blade/trimmer requests in manual and automatic modes
  /// only (idle/stop or a lost mode heartbeat switch them off).
  bool get canUseActuators => mode == 'manual' || isAutonomous;
}
