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

/// Turning (left/right) is sized in rad/s, not by the speed-limit level: at
/// least [turnMinRadS] at any level (below that the motors do not start: 0.3
/// rad/s is only ~0.05 m/s per wheel), up to [turnMaxRadS] on Rápido.
const double turnMinRadS = 0.8;

/// Angular speed of speed 1.0 in the bridge (bridge/move_mapping.py,
/// MAX_ANGULAR_RADS). `speed` of a turn = rad/s / this, so it stays 0..1.
const double turnMaxRadS = 1.5;

/// What goes into robot/cmd/move for a stick position (x, y in -1..1, y < 0 is
/// forward). The dominant axis picks the direction. Forward/back: `speed` is
/// the deflection, clamped to 0..1, times the speed-limit [factor]
/// (0.3 / 0.6 / 1.0). Left/right: the angular speed is
/// max([minTurnRadS], deflection * factor * [turnMaxRadS]) and `speed` is that
/// over [turnMaxRadS]. Inside the dead zone it is ('stop', 0).
({String dir, double speed}) joystickCommand(double x, double y,
    {double factor = 1.0, double minTurnRadS = turnMinRadS}) {
  if (x.abs() < joystickDeadZone && y.abs() < joystickDeadZone) {
    return (dir: 'stop', speed: 0.0);
  }
  if (y.abs() >= x.abs()) {
    return (
      dir: y < 0 ? 'fwd' : 'back',
      speed: y.abs().clamp(0.0, 1.0) * factor,
    );
  }
  final omega = (x.abs().clamp(0.0, 1.0) * factor * turnMaxRadS)
      .clamp(minTurnRadS, turnMaxRadS);
  return (dir: x > 0 ? 'right' : 'left', speed: omega / turnMaxRadS);
}

class JoystickWidget extends StatefulWidget {
  /// Tests only: receives (topic, payload) instead of MqttService.publish.
  final void Function(String topic, String payload)? publishOverride;

  /// False when the robot is not in manual mode: the stick is gray and ignores
  /// touches (the bridge would drop robot/cmd/move anyway).
  final bool modeEnabled;

  const JoystickWidget({super.key, this.publishOverride, this.modeEnabled = true});

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
    final enabled =
        context.watch<ConnectionProvider>().isConnected && widget.modeEnabled;

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
