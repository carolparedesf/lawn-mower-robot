import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';

/// Mode the robot reports in robot/status is "manual": the only one in which
/// the joystick does anything.
bool isManualMode(String mode) => mode == 'manual';

/// Manual / Autónomo / Stop. The highlighted button follows the mode reported
/// in robot/status, not the last button touched.
class ModeButtons extends StatelessWidget {
  const ModeButtons({super.key});

  @override
  Widget build(BuildContext context) {
    final mode = context.watch<RobotStateProvider>().currentStatus.mode;
    final connection = context.read<ConnectionProvider>();
    return Row(
      children: [
        Expanded(
          child: _ModeButton(
            key: const Key('mode_manual'),
            label: 'Manual',
            active: isManualMode(mode),
            onPressed: connection.startManual,
          ),
        ),
        const SizedBox(width: 8),
        Expanded(
          child: _ModeButton(
            key: const Key('mode_auto'),
            label: 'Autónomo',
            active: mode.startsWith('auto_'),
            onPressed: connection.startAutonomous,
          ),
        ),
        const SizedBox(width: 8),
        Expanded(
          child: _ModeButton(
            key: const Key('mode_stop'),
            label: 'Stop',
            active: mode == 'stop' || mode == 'idle',
            danger: true,
            onPressed: connection.sendStop,
          ),
        ),
      ],
    );
  }
}

class _ModeButton extends StatelessWidget {
  final String label;
  final bool active;
  final bool danger;
  final VoidCallback onPressed;

  const _ModeButton({
    super.key,
    required this.label,
    required this.active,
    required this.onPressed,
    this.danger = false,
  });

  @override
  Widget build(BuildContext context) {
    final color = danger ? RoboColors.error : RoboColors.primary;
    return ElevatedButton(
      onPressed: onPressed,
      style: ElevatedButton.styleFrom(
        backgroundColor: active ? color : Colors.white,
        foregroundColor: active ? Colors.white : color,
        side: BorderSide(color: color, width: 1.5),
        elevation: active ? 3 : 0,
        padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 12),
        textStyle: const TextStyle(
            fontFamily: 'Roboto', fontWeight: FontWeight.w700, fontSize: 14),
      ),
      child: FittedBox(fit: BoxFit.scaleDown, child: Text(label)),
    );
  }
}
