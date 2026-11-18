import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/blade_status.dart';
import '../models/robot_status.dart';
import '../providers/actuator_provider.dart';
import '../providers/connection_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';
import '../widgets/battery_display.dart';
import '../widgets/actuator_switch_tile.dart';
import '../widgets/robo_top_bar.dart';

/// "Actuadores" tab — blade/trimmer/charging control, backed by the new
/// robot/cmd/actuator topic + robot/status readback (RobotStatus.blade/
/// trimmer/charging), plus emergency stop.
class ActuatorsScreen extends StatelessWidget {
  final VoidCallback onSettingsTap;

  const ActuatorsScreen({super.key, required this.onSettingsTap});

  /// "Pedido" and "corriendo" are shown separately: a requested motor can be
  /// off because a safety condition is not met.
  String _requestLabel(bool requested, bool running) =>
      'Pedido: ${requested ? 'sí' : 'no'}  ·  Corriendo: ${running ? 'sí' : 'no'}';

  String? _detail(bool requested, bool running, MotorState? motor) {
    if (!requested || running) return null;
    final why = whyNotRunning(motor);
    return why == null ? 'Esperando condiciones…' : 'No corre: $why';
  }

  Future<void> _onForceCut(BuildContext context, ActuatorProvider actuator,
      RobotStatus status, bool value) async {
    if (!value) {
      actuator.setActuators(status, forceCut: false);
      return;
    }
    final ok = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('¿Cortar sin detectar pasto?'),
        content: const Text(
          'La cuchilla podrá encenderse aunque la cámara no vea pasto. '
          'Solo se anula esa condición: el cerco, el robot levantado o '
          'inclinado y la parada de emergencia siguen apagándola.\n\n'
          'Usalo con la zona despejada.',
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(false),
            child: const Text('Cancelar'),
          ),
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(true),
            child: const Text('Forzar corte'),
          ),
        ],
      ),
    );
    if (ok == true) actuator.setActuators(status, forceCut: true);
  }

  @override
  Widget build(BuildContext context) {
    final status = context.watch<RobotStateProvider>().currentStatus;
    final actuator = context.read<ActuatorProvider>();
    final batteryColor = batteryColorFor(status.battery);
    final canUse = status.canUseActuators;
    final motors = parseBladeStatus(status.bladeStatus);

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
                  color: RoboColors.text,
                ),
              ),
              SizedBox(height: 2),
              Text(
                'Control manual de herramientas',
                style: TextStyle(fontSize: 13, color: RoboColors.textMuted),
              ),
            ],
          ),
        ),
        Expanded(
          child: ListView(
            padding: const EdgeInsets.fromLTRB(20, 14, 20, 14),
            children: [
              if (!canUse)
                const _Notice(
                  'Cuchilla y trimmer se pueden pedir solo en modo manual o '
                  'automático.',
                ),
              ActuatorSwitchTile(
                icon: Container(
                  width: 22,
                  height: 22,
                  decoration: BoxDecoration(
                    shape: BoxShape.circle,
                    border: Border.all(
                      color: status.bladeRunning
                          ? RoboColors.primary
                          : RoboColors.disabled,
                      width: 3,
                    ),
                  ),
                ),
                iconBg: status.bladeRunning
                    ? RoboColors.statusBar
                    : RoboColors.border,
                title: 'Cuchilla de corte',
                statusLabel: _requestLabel(status.blade, status.bladeRunning),
                statusColor: status.bladeRunning
                    ? RoboColors.primary
                    : RoboColors.textMuted,
                detail: _detail(status.blade, status.bladeRunning,
                    motors['cutter']),
                detailColor: RoboColors.warning,
                value: status.blade,
                onChanged: canUse
                    ? (v) => actuator.setActuators(status, blade: v)
                    : null,
              ),
              const SizedBox(height: 14),
              ActuatorSwitchTile(
                icon: Container(
                  width: 22,
                  height: 14,
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(7),
                    border: Border.all(
                      color: status.trimmerRunning
                          ? RoboColors.warning
                          : RoboColors.disabled,
                      width: 3,
                    ),
                  ),
                ),
                iconBg: status.trimmerRunning
                    ? RoboColors.warningChip
                    : RoboColors.border,
                title: 'Trimmer / Bordeadora',
                statusLabel:
                    _requestLabel(status.trimmer, status.trimmerRunning),
                statusColor: status.trimmerRunning
                    ? RoboColors.warning
                    : RoboColors.textMuted,
                detail: _detail(status.trimmer, status.trimmerRunning,
                    motors['trimmer']),
                detailColor: RoboColors.warning,
                value: status.trimmer,
                onChanged: canUse
                    ? (v) => actuator.setActuators(status, trimmer: v)
                    : null,
              ),
              const SizedBox(height: 14),
              ActuatorSwitchTile(
                icon: const Icon(Icons.grass, color: RoboColors.error),
                iconBg: RoboColors.errorChip,
                title: 'Cortar sin detectar pasto',
                statusLabel: status.forceCut ? 'Forzado' : 'Desactivado',
                statusColor: status.forceCut
                    ? RoboColors.error
                    : RoboColors.textMuted,
                detail: 'Anula solo la detección de pasto. Cerco, levantado, '
                    'inclinación y emergencia siguen activos.',
                value: status.forceCut,
                onChanged: canUse
                    ? (v) => _onForceCut(context, actuator, status, v)
                    : null,
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
                            color: RoboColors.text,
                          ),
                        ),
                        Text(
                          batteryLabel(status.battery),
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
                        value: (status.battery ?? 0).clamp(0, 100) / 100,
                        minHeight: 14,
                        backgroundColor: RoboColors.border,
                        color: batteryColor,
                      ),
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
                              color: RoboColors.textMuted),
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

class _Notice extends StatelessWidget {
  final String text;
  const _Notice(this.text);

  @override
  Widget build(BuildContext context) {
    return Container(
      margin: const EdgeInsets.only(bottom: 14),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: RoboColors.warningChip,
        borderRadius: BorderRadius.circular(14),
      ),
      child: Row(
        children: [
          const Icon(Icons.info_outline,
              size: 16, color: RoboColors.textSecondary),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              text,
              style: const TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w700,
                color: RoboColors.textSecondary,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
