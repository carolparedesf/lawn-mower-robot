import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:provider/provider.dart';
import '../config/mqtt_config.dart';
import '../providers/connection_provider.dart';
import '../services/mqtt_service.dart';

/// Visual size of the joystick (the logic is independent of it).
const double joystickBaseSize = 150;
const double joystickStickSize = 37.5;

class JoystickWidget extends StatefulWidget {
  const JoystickWidget({super.key});

  @override
  State<JoystickWidget> createState() => _JoystickWidgetState();
}

class _JoystickWidgetState extends State<JoystickWidget> {
  Timer? _throttle;
  String _lastDir = 'stop';
  double _lastSpeed = 0.0;

  void _onMove(StickDragDetails details) {
    final x = details.x;
    final y = details.y;

    String dir;
    double speed;

    if (x.abs() < 0.2 && y.abs() < 0.2) {
      dir = 'stop';
      speed = 0.0;
    } else if (y.abs() >= x.abs()) {
      dir = y < 0 ? 'fwd' : 'back';
      speed = y.abs();
    } else {
      dir = x > 0 ? 'right' : 'left';
      speed = x.abs();
    }

    _lastDir = dir;
    _lastSpeed = speed.clamp(0.0, 1.0);

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
