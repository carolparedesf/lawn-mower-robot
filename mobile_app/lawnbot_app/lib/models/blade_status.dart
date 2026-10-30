/// Parser and Spanish text for the robot's /blade/status, forwarded in
/// robot/status as `blade_status`, e.g.
///   "cutter=OFF(sin_pasto,levantado) | trimmer=ON duty=0.30"
class MotorState {
  final bool on;
  final double? duty;

  /// Reasons the motor is off (robot codes), empty when on.
  final List<String> reasons;

  const MotorState({required this.on, this.duty, this.reasons = const []});

  /// Reasons other than "nobody asked for it".
  List<String> get blockingReasons =>
      reasons.where((r) => r != 'sin_pedido').toList();
}

/// Motor name ("cutter", "trimmer") -> state. Empty map if null/unparsable.
Map<String, MotorState> parseBladeStatus(String? text) {
  final result = <String, MotorState>{};
  if (text == null || text.isEmpty) return result;
  for (final part in text.split('|')) {
    final m = RegExp(r'^\s*(\w+)=(ON|OFF)(?:\(([^)]*)\))?(?:\s+duty=([0-9.]+))?\s*$')
        .firstMatch(part);
    if (m == null) continue;
    final reasons = (m.group(3) ?? '')
        .split(',')
        .map((r) => r.trim())
        .where((r) => r.isNotEmpty)
        .toList();
    result[m.group(1)!] = MotorState(
      on: m.group(2) == 'ON',
      duty: m.group(4) == null ? null : double.tryParse(m.group(4)!),
      reasons: reasons,
    );
  }
  return result;
}

const Map<String, String> _reasonText = {
  'estop': 'parada de emergencia',
  'geofence_sin_dato': 'sin dato del cerco',
  'fuera_del_cerco': 'fuera del cerco',
  'bumpers_sin_dato': 'sin dato de los bumpers',
  'levantado': 'robot levantado',
  'imu_sin_dato': 'sin dato de inclinación',
  'volcado': 'robot inclinado',
  'pasadas_desconocidas': 'sin mapa de cobertura',
  'celda_con_demasiadas_pasadas': 'zona ya cortada',
  'pasto_sin_dato': 'sin dato de pasto',
  'sin_pasto': 'no detecta pasto',
  'sin_pedido': 'sin pedido',
};

String bladeReasonText(String code) => _reasonText[code] ?? code;

/// One-line explanation of why a requested motor is not running, or null.
String? whyNotRunning(MotorState? state) {
  if (state == null || state.on) return null;
  final reasons = state.blockingReasons;
  if (reasons.isEmpty) return null;
  return reasons.map(bladeReasonText).join(', ');
}
