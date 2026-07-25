import 'package:flutter/material.dart';
import '../theme/robo_theme.dart';

/// Reusable icon+title+status+switch row for actuators_screen.dart,
/// mirroring the mockup's blade/trimmer cards.
class ActuatorSwitchTile extends StatelessWidget {
  final Widget icon;
  final Color iconBg;
  final String title;
  final String statusLabel;
  final Color statusColor;
  final bool value;
  final ValueChanged<bool> onChanged;

  const ActuatorSwitchTile({
    super.key,
    required this.icon,
    required this.iconBg,
    required this.title,
    required this.statusLabel,
    required this.statusColor,
    required this.value,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(20),
        boxShadow: const [
          BoxShadow(
            color: Color(0x0D000000),
            blurRadius: 8,
            offset: Offset(0, 2),
          ),
        ],
      ),
      child: Row(
        children: [
          Container(
            width: 48,
            height: 48,
            decoration: BoxDecoration(
              color: iconBg,
              borderRadius: BorderRadius.circular(14),
            ),
            child: Center(child: icon),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: const TextStyle(
                    fontWeight: FontWeight.w700,
                    fontSize: 15,
                    color: RoboColors.darkGreen,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  statusLabel,
                  style: TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w700,
                    color: statusColor,
                  ),
                ),
              ],
            ),
          ),
          Switch(value: value, onChanged: onChanged),
        ],
      ),
    );
  }
}
