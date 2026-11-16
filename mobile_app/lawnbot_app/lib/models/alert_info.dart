/// Spanish text and severity for the `alert` field of robot/status.
///
/// The robot sends a code: either a safety/navigation alert (kept while the
/// condition lasts) or, for ~5 s, an event from /robot/events (lifted,
/// low_battery, ...). "none" means no alert.
enum AlertSeverity { info, warning, error }

class AlertInfo {
  final String code;
  final String text;
  final AlertSeverity severity;

  /// True for one-shot robot events (they disappear by themselves after ~5 s).
  final bool isEvent;

  const AlertInfo(this.code, this.text, this.severity, {this.isEvent = false});
}

const String alertNone = 'none';

const Map<String, AlertInfo> _table = {
  // Safety gate
  'estop': AlertInfo('estop', 'Parada de emergencia activa', AlertSeverity.error),
  'imu_lost': AlertInfo('imu_lost',
      'Se perdió el IMU: modos automáticos detenidos', AlertSeverity.error),
  'sensor_timeout': AlertInfo('sensor_timeout',
      'Sin datos de los sensores: el robot no avanza', AlertSeverity.error),
  'bumper_front': AlertInfo(
      'bumper_front', 'Choque en el frente: el robot se detuvo', AlertSeverity.error),
  'bumper_back': AlertInfo(
      'bumper_back', 'Choque atrás: el robot se detuvo', AlertSeverity.error),
  'obstacle_stop': AlertInfo('obstacle_stop',
      'Obstáculo muy cerca: el robot se detuvo', AlertSeverity.error),
  'obstacle_near': AlertInfo('obstacle_near',
      'Obstáculo cerca: el robot reduce la velocidad', AlertSeverity.warning),
  // Fence and position
  'geofence_breach':
      AlertInfo('geofence_breach', 'El robot salió del cerco', AlertSeverity.error),
  'outside_fence':
      AlertInfo('outside_fence', 'El robot salió del cerco', AlertSeverity.error),
  'geofence_not_ready': AlertInfo('geofence_not_ready',
      'El cerco todavía no está listo en el robot', AlertSeverity.warning),
  'no_pose': AlertInfo(
      'no_pose', 'El robot no conoce su posición', AlertSeverity.warning),
  'tf_lost': AlertInfo('tf_lost',
      'Se perdió la posición del robot (TF)', AlertSeverity.error),
  // Navigation
  'no_progress': AlertInfo('no_progress',
      'El robot no avanza hacia su destino', AlertSeverity.warning),
  'local_minimum': AlertInfo('local_minimum',
      'Atascado entre obstáculos: el robot probará otra maniobra', AlertSeverity.warning),
  'perimeter_blocked': AlertInfo('perimeter_blocked',
      'No puede seguir el borde: bloqueado', AlertSeverity.error),
  'perimeter_lost': AlertInfo('perimeter_lost',
      'Perdió el borde y no lo encuentra', AlertSeverity.error),
  'perimeter_misconfigured': AlertInfo('perimeter_misconfigured',
      'Seguimiento de borde mal configurado en el robot', AlertSeverity.error),
  'random_blocked': AlertInfo('random_blocked',
      'Atascado en el recorrido aleatorio', AlertSeverity.warning),
  'coverage_error': AlertInfo('coverage_error',
      'No se pudo armar la ruta de corte con este cerco', AlertSeverity.error),
  'auto_start_timeout': AlertInfo('auto_start_timeout',
      'El modo automático no pudo iniciar a tiempo', AlertSeverity.warning),
  // One-shot events (/robot/events)
  'lifted': AlertInfo('lifted', 'El robot fue levantado', AlertSeverity.error,
      isEvent: true),
  'low_battery': AlertInfo('low_battery', 'Batería baja', AlertSeverity.warning,
      isEvent: true),
  'cutting_started': AlertInfo(
      'cutting_started', 'Corte iniciado', AlertSeverity.info,
      isEvent: true),
  'navigation_started': AlertInfo(
      'navigation_started', 'Navegación iniciada', AlertSeverity.info,
      isEvent: true),
  'perimeter_started': AlertInfo(
      'perimeter_started', 'Siguiendo el borde del cerco', AlertSeverity.info,
      isEvent: true),
};

/// Description for an alert code, or null for "none"/empty. Unknown codes get
/// a generic warning that still shows the code so nothing is silently hidden.
AlertInfo? describeAlert(String? code) {
  if (code == null || code.isEmpty || code == alertNone) return null;
  return _table[code] ??
      AlertInfo(code, 'Alerta del robot: $code', AlertSeverity.warning);
}

/// Codes with a translation (for tests and docs).
Iterable<String> get knownAlertCodes => _table.keys;
