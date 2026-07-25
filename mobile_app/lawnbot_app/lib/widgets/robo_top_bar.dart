import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';

/// Replaces the stock AppBar. Logo+title+settings row, then a
/// connection/battery status row — folds in the old connection_indicator.dart.
class RoboTopBar extends StatelessWidget implements PreferredSizeWidget {
  final VoidCallback onSettingsTap;

  const RoboTopBar({super.key, required this.onSettingsTap});

  @override
  Size get preferredSize => const Size.fromHeight(90);

  @override
  Widget build(BuildContext context) {
    final connection = context.watch<ConnectionProvider>();
    final battery = context.watch<RobotStateProvider>().currentStatus.battery;
    final batteryColor = battery <= 20
        ? RoboColors.error
        : battery <= 45
            ? RoboColors.amberAccent
            : RoboColors.primaryGreen;

    return Column(
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(20, 18, 20, 10),
          child: Row(
            children: [
              Container(
                width: 32,
                height: 32,
                decoration: BoxDecoration(
                  color: RoboColors.primaryGreen,
                  borderRadius: BorderRadius.circular(10),
                  boxShadow: [
                    BoxShadow(
                      color: RoboColors.primaryGreen.withValues(alpha: 0.4),
                      blurRadius: 10,
                      offset: const Offset(0, 4),
                    ),
                  ],
                ),
                child: Center(
                  child: Container(
                    width: 13,
                    height: 13,
                    decoration: const BoxDecoration(
                      color: RoboColors.background,
                      shape: BoxShape.circle,
                    ),
                  ),
                ),
              ),
              const SizedBox(width: 10),
              const Text(
                'RobotControl',
                style: TextStyle(
                  fontWeight: FontWeight.w700,
                  fontSize: 17,
                  color: RoboColors.darkGreen,
                ),
              ),
              const Spacer(),
              InkWell(
                onTap: onSettingsTap,
                borderRadius: BorderRadius.circular(12),
                child: Container(
                  width: 34,
                  height: 34,
                  decoration: BoxDecoration(
                    color: RoboColors.border,
                    borderRadius: BorderRadius.circular(12),
                  ),
                  child: const Icon(Icons.settings,
                      size: 18, color: RoboColors.textTertiary),
                ),
              ),
            ],
          ),
        ),
        Container(
          width: double.infinity,
          padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 7),
          color: RoboColors.selectedSurface,
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Row(
                children: [
                  Container(
                    width: 7,
                    height: 7,
                    decoration: BoxDecoration(
                      color: connection.isConnected
                          ? RoboColors.primaryGreen
                          : connection.isConnecting
                              ? RoboColors.amberAccent
                              : RoboColors.error,
                      shape: BoxShape.circle,
                    ),
                  ),
                  const SizedBox(width: 6),
                  Text(
                    connection.isConnected
                        ? 'Conectado'
                        : connection.isConnecting
                            ? 'Conectando...'
                            : 'Desconectado',
                    style: const TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w700,
                      color: RoboColors.darkGreen,
                    ),
                  ),
                ],
              ),
              Row(
                children: [
                  Container(
                    width: 18,
                    height: 10,
                    padding: const EdgeInsets.all(1.5),
                    decoration: BoxDecoration(
                      border: Border.all(color: batteryColor, width: 1.5),
                      borderRadius: BorderRadius.circular(3),
                    ),
                    child: FractionallySizedBox(
                      alignment: Alignment.centerLeft,
                      widthFactor: (battery.clamp(0, 100)) / 100,
                      child: DecoratedBox(
                        decoration: BoxDecoration(
                          color: batteryColor,
                          borderRadius: BorderRadius.circular(1),
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(width: 6),
                  Text(
                    '$battery%',
                    style: const TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w700,
                      color: RoboColors.textTertiary,
                    ),
                  ),
                ],
              ),
            ],
          ),
        ),
      ],
    );
  }
}
