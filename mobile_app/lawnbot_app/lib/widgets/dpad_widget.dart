import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../config/mqtt_config.dart';
import '../providers/connection_provider.dart';
import '../services/mqtt_service.dart';
import '../theme/robo_theme.dart';

const double _dpadSpeed = 0.6;

/// Alternate manual-control input to the joystick. Publishes the same
/// dir/speed contract to robot/cmd/move: press-and-hold sends the
/// direction, release sends stop.
class DpadWidget extends StatelessWidget {
  const DpadWidget({super.key});

  void _publish(String dir, double speed) {
    final payload = jsonEncode({'dir': dir, 'speed': speed});
    MqttService.instance.publish(MqttConfig.topicCmdMove, payload);
  }

  @override
  Widget build(BuildContext context) {
    final enabled = context.watch<ConnectionProvider>().isConnected;

    return Opacity(
      opacity: enabled ? 1.0 : 0.4,
      child: IgnorePointer(
        ignoring: !enabled,
        child: SizedBox(
          width: 108,
          height: 108,
          child: GridView.count(
            crossAxisCount: 3,
            mainAxisSpacing: 6,
            crossAxisSpacing: 6,
            physics: const NeverScrollableScrollPhysics(),
            children: [
              const SizedBox.shrink(),
              _DpadButton(label: '▲', dir: 'fwd', onPublish: _publish),
              const SizedBox.shrink(),
              _DpadButton(label: '◀', dir: 'left', onPublish: _publish),
              DecoratedBox(
                decoration: BoxDecoration(
                  color: RoboColors.selectedSurfaceAlt,
                  borderRadius: BorderRadius.circular(12),
                ),
              ),
              _DpadButton(label: '▶', dir: 'right', onPublish: _publish),
              const SizedBox.shrink(),
              _DpadButton(label: '▼', dir: 'back', onPublish: _publish),
              const SizedBox.shrink(),
            ],
          ),
        ),
      ),
    );
  }
}

class _DpadButton extends StatefulWidget {
  final String label;
  final String dir;
  final void Function(String dir, double speed) onPublish;

  const _DpadButton({
    required this.label,
    required this.dir,
    required this.onPublish,
  });

  @override
  State<_DpadButton> createState() => _DpadButtonState();
}

class _DpadButtonState extends State<_DpadButton> {
  Timer? _repeat;

  void _start() {
    widget.onPublish(widget.dir, _dpadSpeed);
    // El ESP32 corta el movimiento si no recibe un cmd_vel nuevo en 500ms
    // (CMD_WATCHDOG_MS en firmware/src/main.cpp) — hay que seguir
    // mandando el comando mientras el botón esté presionado.
    _repeat = Timer.periodic(const Duration(milliseconds: 150), (_) {
      widget.onPublish(widget.dir, _dpadSpeed);
    });
  }

  void _stop() {
    _repeat?.cancel();
    _repeat = null;
    widget.onPublish('stop', 0.0);
  }

  @override
  void dispose() {
    _repeat?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTapDown: (_) => _start(),
      onTapUp: (_) => _stop(),
      onTapCancel: _stop,
      child: DecoratedBox(
        decoration: BoxDecoration(
          color: RoboColors.primaryGreen,
          borderRadius: BorderRadius.circular(12),
        ),
        child: Center(
          child: Text(
            widget.label,
            style: const TextStyle(
              color: Colors.white,
              fontWeight: FontWeight.w800,
              fontSize: 16,
            ),
          ),
        ),
      ),
    );
  }
}
