import 'package:flutter_test/flutter_test.dart';
import 'package:robotmapapp/models/alert_info.dart';

void main() {
  test('none and empty mean no alert', () {
    expect(describeAlert('none'), isNull);
    expect(describeAlert(''), isNull);
    expect(describeAlert(null), isNull);
  });

  test('every robot alert code has Spanish text and severity', () {
    const codes = [
      // /nav/safety_alert (safety_logic.ALERT_PRIORITY + potential_field)
      'sensor_timeout', 'bumper_front', 'bumper_back', 'obstacle_stop',
      'estop', 'geofence_breach', 'tf_lost', 'no_progress', 'perimeter_blocked',
      'perimeter_lost', 'perimeter_misconfigured', 'random_blocked',
      'local_minimum', 'geofence_not_ready', 'coverage_error', 'no_pose',
      'auto_start_timeout', 'obstacle_near', 'imu_lost', 'outside_fence',
      // /robot/events
      'lifted', 'cutting_started', 'navigation_started', 'perimeter_started',
      'low_battery',
    ];
    for (final c in codes) {
      final info = describeAlert(c);
      expect(info, isNotNull, reason: c);
      expect(info!.code, c);
      expect(info.text, isNot(contains(c)), reason: 'translated, not the raw code: $c');
      expect(info.text.isNotEmpty, isTrue);
    }
    expect(knownAlertCodes.toSet().containsAll(codes), isTrue);
  });

  test('geofence_breach is a soft amber notice; outside_fence is the red one', () {
    final near = describeAlert('geofence_breach')!;
    expect(near.text, 'Cerca del borde del cerco');
    expect(near.severity, AlertSeverity.warning);
    final outside = describeAlert('outside_fence')!;
    expect(outside.text, 'El robot salió del cerco');
    expect(outside.severity, AlertSeverity.error);
  });

  test('estop is an error; obstacle_near only warns', () {
    expect(describeAlert('estop')!.severity, AlertSeverity.error);
    expect(describeAlert('obstacle_near')!.severity, AlertSeverity.warning);
  });

  test('imu_lost and outside_fence have their Spanish text and are errors', () {
    final imu = describeAlert('imu_lost')!;
    expect(imu.text, 'Se perdió el IMU: modos automáticos detenidos');
    expect(imu.severity, AlertSeverity.error);
    expect(imu.isEvent, isFalse);
    final fence = describeAlert('outside_fence')!;
    expect(fence.text, 'El robot salió del cerco');
    expect(fence.severity, AlertSeverity.error);
    expect(fence.isEvent, isFalse);
  });

  test('events are flagged as one-shot', () {
    for (final c in ['lifted', 'low_battery', 'cutting_started', 'navigation_started', 'perimeter_started']) {
      expect(describeAlert(c)!.isEvent, isTrue, reason: c);
    }
    expect(describeAlert('estop')!.isEvent, isFalse);
  });

  test('an unknown code is still shown (never silently dropped)', () {
    final info = describeAlert('algo_nuevo');
    expect(info, isNotNull);
    expect(info!.text, contains('algo_nuevo'));
    expect(info.severity, AlertSeverity.warning);
  });
}
