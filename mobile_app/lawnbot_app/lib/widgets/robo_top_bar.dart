import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';
import 'alert_banner.dart';
import 'battery_display.dart';

/// Top bar of the first (pink) version: pink bar with the title, the white
/// connection chip and the settings gear (it also paints the status-bar area),
/// then a light-pink strip with the robot's mode and battery, then the alert
/// banner when there is one.
class RoboTopBar extends StatelessWidget {
  final VoidCallback onSettingsTap;

  const RoboTopBar({super.key, required this.onSettingsTap});

  @override
  Widget build(BuildContext context) {
    final connection = context.watch<ConnectionProvider>();
    final status = context.watch<RobotStateProvider>().currentStatus;
    final battery = status.battery;
    final batteryColor = batteryColorFor(battery);
    final top = MediaQuery.of(context).padding.top;

    return AnnotatedRegion<SystemUiOverlayStyle>(
      value: SystemUiOverlayStyle.light.copyWith(
        statusBarColor: Colors.transparent,
        statusBarIconBrightness: Brightness.light,
      ),
      child: Column(
        children: [
          Container(
            key: const Key('top_bar'),
            color: RoboColors.appBar,
            padding: EdgeInsets.fromLTRB(16, top + 12, 4, 12),
            child: Row(
              children: [
                const Expanded(
                  child: Text(
                    'RobotControl',
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                    style: TextStyle(
                      fontSize: 22,
                      fontWeight: FontWeight.w400,
                      color: Colors.white,
                    ),
                  ),
                ),
                _ConnectionChip(
                  connected: connection.isConnected,
                  connecting: connection.isConnecting,
                ),
                IconButton(
                  key: const Key('settings_button'),
                  tooltip: 'Conexión',
                  onPressed: onSettingsTap,
                  icon: const Icon(Icons.settings, color: Colors.white),
                ),
              ],
            ),
          ),
          Container(
            key: const Key('status_strip'),
            width: double.infinity,
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
            decoration: const BoxDecoration(
              color: RoboColors.statusBar,
              border: Border(bottom: BorderSide(color: RoboColors.borderStrong)),
            ),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Row(
                  children: [
                    const Icon(Icons.settings_remote,
                        size: 22, color: RoboColors.text),
                    const SizedBox(width: 8),
                    Text(
                      'Modo: ${status.mode}',
                      key: const Key('status_mode'),
                      style: const TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w700,
                        color: RoboColors.text,
                      ),
                    ),
                  ],
                ),
                Row(
                  children: [
                    Icon(
                      (battery ?? 0) <= 20
                          ? Icons.battery_alert
                          : Icons.battery_full,
                      size: 22,
                      color: batteryColor,
                    ),
                    const SizedBox(width: 4),
                    Text(
                      batteryLabel(battery),
                      key: const Key('status_battery'),
                      style: const TextStyle(
                        fontSize: 16,
                        color: RoboColors.text,
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
          const AlertBanner(),
        ],
      ),
    );
  }
}

/// White chip with a colored dot: green connected, orange connecting, red not.
class _ConnectionChip extends StatelessWidget {
  final bool connected;
  final bool connecting;

  const _ConnectionChip({required this.connected, required this.connecting});

  @override
  Widget build(BuildContext context) {
    final color = connected
        ? RoboColors.success
        : connecting
            ? RoboColors.warning
            : RoboColors.error;
    return Container(
      key: const Key('connection_chip'),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 7),
      decoration: BoxDecoration(
        color: RoboColors.background,
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: RoboColors.border),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 12,
            height: 12,
            decoration: BoxDecoration(color: color, shape: BoxShape.circle),
          ),
          const SizedBox(width: 12),
          Text(
            connected
                ? 'Conectado'
                : connecting
                    ? 'Conectando...'
                    : 'Desconectado',
            style: const TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w500,
              color: RoboColors.textSecondary,
            ),
          ),
        ],
      ),
    );
  }
}
