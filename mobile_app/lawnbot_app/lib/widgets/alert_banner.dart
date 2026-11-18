import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/alert_info.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';

/// Full-width banner with the robot's current alert (robot/status.alert),
/// translated to Spanish. Hidden when there is none.
class AlertBanner extends StatelessWidget {
  const AlertBanner({super.key});

  @override
  Widget build(BuildContext context) {
    final code = context.watch<RobotStateProvider>().currentStatus.alert;
    final info = describeAlert(code);
    if (info == null) return const SizedBox.shrink();

    final Color bg;
    final Color fg;
    final IconData icon;
    switch (info.severity) {
      case AlertSeverity.error:
        bg = RoboColors.errorChip;
        fg = RoboColors.error;
        icon = Icons.error_outline;
      case AlertSeverity.warning:
        bg = RoboColors.warningChip;
        fg = RoboColors.textSecondary;
        icon = Icons.warning_amber_rounded;
      case AlertSeverity.info:
        bg = RoboColors.statusBar;
        fg = RoboColors.textSecondary;
        icon = Icons.info_outline;
    }

    return Container(
      key: const Key('alert_banner'),
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 8),
      color: bg,
      child: Row(
        children: [
          Icon(icon, size: 16, color: fg),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              info.text,
              style: TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w700,
                color: fg,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
