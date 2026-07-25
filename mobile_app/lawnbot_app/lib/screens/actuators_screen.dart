import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/robot_status.dart';
import '../providers/actuator_provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';
import '../widgets/actuator_switch_tile.dart';
import '../widgets/robo_top_bar.dart';

/// "Actuadores" tab — blade/trimmer/charging control, backed by the new
/// robot/cmd/actuator topic + robot/status readback (RobotStatus.blade/
/// trimmer/charging), plus emergency stop.
class ActuatorsScreen extends StatelessWidget {
  final VoidCallback onSettingsTap;

  const ActuatorsScreen({super.key, required this.onSettingsTap});

  String _timeRemaining(RobotStatus status) {
    if (status.charging) return 'Cargando…';
    final drainRate = (status.blade ? 1 : 0) +
        (status.trimmer ? 1 : 0) +
        (status.isAutonomous ? 1 : 0);
    if (drainRate == 0) return '${(status.battery / 3).round()} h aprox.';
    final hours = (status.battery / (drainRate * 4)).round();
    return '${hours < 1 ? 1 : hours} h aprox.';
  }

  @override
  Widget build(BuildContext context) {
    final status = context.watch<RobotStateProvider>().currentStatus;
    final actuator = context.read<ActuatorProvider>();
    final batteryColor = status.battery <= 20
        ? RoboColors.error
        : status.battery <= 45
            ? RoboColors.amberAccent
            : RoboColors.primaryGreen;

    return Column(
      children: [
        RoboTopBar(onSettingsTap: onSettingsTap),
        const Padding(
          padding: EdgeInsets.fromLTRB(20, 14, 20, 4),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'Actuadores',
                style: TextStyle(
                  fontWeight: FontWeight.w700,
                  fontSize: 18,
                  color: RoboColors.darkGreen,
                ),
              ),
              SizedBox(height: 2),
              Text(
                'Control manual de herramientas',
                style: TextStyle(fontSize: 13, color: RoboColors.textSecondary),
              ),
            ],
          ),
        ),
        Expanded(
          child: ListView(
            padding: const EdgeInsets.fromLTRB(20, 14, 20, 14),
            children: [
              ActuatorSwitchTile(
                icon: Container(
                  width: 22,
                  height: 22,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    border: Border.all(
                      color: status.blade
                          ? RoboColors.primaryGreen
                          : RoboColors.disabled,
                      width: 3,
                    ),
                  ),
                ),
                iconBg: status.blade
                    ? RoboColors.selectedSurfaceAlt
                    : RoboColors.border,
                title: 'Cuchilla de corte',
                statusLabel: status.blade ? 'Encendida' : 'Apagada',
                statusColor: status.blade
                    ? RoboColors.primaryGreen
                    : RoboColors.textSecondary,
                value: status.blade,
                onChanged: (v) => actuator.setActuators(status, blade: v),
              ),
              const SizedBox(height: 14),
              ActuatorSwitchTile(
                icon: Container(
                  width: 22,
                  height: 14,
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(7),
                    border: Border.all(
                      color: status.trimmer
                          ? RoboColors.amberAccent
                          : RoboColors.disabled,
                      width: 3,
                    ),
                  ),
                ),
                iconBg:
                    status.trimmer ? RoboColors.amberChip : RoboColors.border,
                title: 'Trimmer / Bordeadora',
                statusLabel: status.trimmer ? 'Encendido' : 'Apagado',
                statusColor: status.trimmer
                    ? RoboColors.amberAccent
                    : RoboColors.textSecondary,
                value: status.trimmer,
                onChanged: (v) => actuator.setActuators(status, trimmer: v),
              ),
              const SizedBox(height: 14),
              Container(
                padding: const EdgeInsets.all(18),
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
                child: Column(
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        const Text(
                          'Batería',
                          style: TextStyle(
                            fontWeight: FontWeight.w700,
                            fontSize: 15,
                            color: RoboColors.darkGreen,
                          ),
                        ),
                        Text(
                          '${status.battery}%',
                          style: TextStyle(
                            fontSize: 22,
                            fontWeight: FontWeight.w800,
                            color: batteryColor,
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 10),
                    ClipRRect(
                      borderRadius: BorderRadius.circular(8),
                      child: LinearProgressIndicator(
                        value: status.battery.clamp(0, 100) / 100,
                        minHeight: 14,
                        backgroundColor: RoboColors.border,
                        color: batteryColor,
                      ),
                    ),
                    const SizedBox(height: 10),
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        const Text('Autonomía estimada',
                            style: TextStyle(
                                fontSize: 12,
                                color: RoboColors.textSecondary)),
                        Text(_timeRemaining(status),
                            style: const TextStyle(
                                fontSize: 12,
                                fontWeight: FontWeight.w700,
                                color: RoboColors.textTertiary)),
                      ],
                    ),
                    const Divider(height: 24),
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        Text(
                          status.charging
                              ? 'Cargando batería'
                              : 'Conectar a carga',
                          style: const TextStyle(
                              fontSize: 12,
                              color: RoboColors.textSecondary),
                        ),
                        Switch(
                          value: status.charging,
                          onChanged: (v) =>
                              actuator.setActuators(status, charging: v),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 14),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: () {
                    context.read<ConnectionProvider>().setMode('stop');
                    actuator.emergencyStop();
                  },
                  style: ElevatedButton.styleFrom(
                    backgroundColor: RoboColors.error,
                    shadowColor: RoboColors.error.withValues(alpha: 0.35),
                  ),
                  icon: const Icon(Icons.crop_square, size: 14),
                  label: const Text('Parada de emergencia'),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }
}
