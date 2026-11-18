import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import '../config/mqtt_config.dart';
import '../providers/connection_provider.dart';
import '../providers/speed_limit_provider.dart';
import '../services/mqtt_service.dart';

/// Visual size of the joystick (the logic is independent of it).
const double joystickBaseSize = 150;
const double joystickStickSize = 37.5;

/// Joystick deflection below this on both axes is "stop" (dead zone).
const double joystickDeadZone = 0.2;

/// What goes into robot/cmd/move for a stick position (x, y in -1..1, y < 0 is
/// forward). The dominant axis picks the direction; its deflection, clamped to
/// 0..1 and multiplied by the speed-limit [factor] (0.3 / 0.6 / 1.0), is the
/// `speed`. Inside the dead zone it is ('stop', 0).
({String dir, double speed}) joystickCommand(double x, double y,
    {double factor = 1.0}) {
  if (x.abs() < joystickDeadZone && y.abs() < joystickDeadZone) {
    return (dir: 'stop', speed: 0.0);
  }
  final String dir;
  final double raw;
  if (y.abs() >= x.abs()) {
    dir = y < 0 ? 'fwd' : 'back';
    raw = y.abs();
  } else {
    dir = x > 0 ? 'right' : 'left';
    raw = x.abs();
  }
  return (dir: dir, speed: raw.clamp(0.0, 1.0) * factor);
}

class JoystickWidget extends StatefulWidget {
  /// Tests only: receives (topic, payload) instead of MqttService.publish.
  final void Function(String topic, String payload)? publishOverride;

  const JoystickWidget({super.key, this.publishOverride});

  @override
  State<JoystickWidget> createState() => _JoystickWidgetState();
}

class _JoystickWidgetState extends State<JoystickWidget> {
  Timer? _throttle;
  String _lastDir = 'stop';
  double _lastSpeed = 0.0;

  void _onMove(StickDragDetails details) {
    final cmd = joystickCommand(details.x, details.y,
        factor: context.read<SpeedLimitProvider>().factor);
    _lastDir = cmd.dir;
    _lastSpeed = cmd.speed;

    if (_throttle != null) return;
    _throttle = Timer(const Duration(milliseconds: 50), () {
      _throttle = null;
      _publish();
    });
  }

  void _onEnd() {
    _throttle?.cancel();
    _throttle = null;
    _lastDir = 'stop';
    _lastSpeed = 0.0;
    _publish();
  }

  void _publish() {
    if (!context.read<ConnectionProvider>().isConnected) return;
    final payload = jsonEncode({'dir': _lastDir, 'speed': _lastSpeed});
    final override = widget.publishOverride;
    if (override != null) {
      override(MqttConfig.topicCmdMove, payload);
      return;
    }
    MqttService.instance.publish(MqttConfig.topicCmdMove, payload);
  }

  @override
  void dispose() {
    _throttle?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final enabled = context.watch<ConnectionProvider>().isConnected;

    return Opacity(
      opacity: enabled ? 1.0 : 0.4,
      child: IgnorePointer(
        ignoring: !enabled,
        child: Joystick(
          mode: JoystickMode.all,
          listener: _onMove,
          onStickDragEnd: _onEnd,
          // Pre-redesign look: flutter_joystick's own defaults (dark gray disc
          // with double arrows, light ring, blue glowing stick), scaled from
          // the package's 200/50 to joystickBaseSize / joystickStickSize.
          base: const JoystickBase(size: joystickBaseSize),
          stick: const JoystickStick(size: joystickStickSize),
        ),
      ),
    );
  }
}
