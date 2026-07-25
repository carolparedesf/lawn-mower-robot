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
          width: 138,
          height: 138,
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

class _DpadButton extends StatelessWidget {
  final String label;
  final String dir;
  final void Function(String dir, double speed) onPublish;

  const _DpadButton({
    required this.label,
    required this.dir,
    required this.onPublish,
  });

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTapDown: (_) => onPublish(dir, _dpadSpeed),
      onTapUp: (_) => onPublish('stop', 0.0),
      onTapCancel: () => onPublish('stop', 0.0),
      child: DecoratedBox(
        decoration: BoxDecoration(
          color: RoboColors.primaryGreen,
          borderRadius: BorderRadius.circular(12),
        ),
        child: Center(
          child: Text(
            label,
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
