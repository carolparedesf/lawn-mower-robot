import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/geofence.dart';
import '../providers/geofence_provider.dart';
import '../providers/robot_state_provider.dart';
import '../theme/robo_theme.dart';

/// User-facing explanation of why the polygon cannot be sent.
String geofenceProblemMessage(GeofenceProblem problem) {
  switch (problem) {
    case GeofenceProblem.tooFewPoints:
      return 'Faltan puntos: el cerco necesita al menos ${Geofence.minPoints}.';
    case GeofenceProblem.selfIntersecting:
      return 'El cerco se cruza a sí mismo. Deshacé el último punto o '
          'limpiá y volvé a marcarlo sin cruzar los lados.';
    case GeofenceProblem.notConvex:
      return 'El cerco tiene una esquina hacia adentro (no es convexo). '
          'El robot solo acepta cercos convexos: movelo para que todas las '
          'esquinas apunten hacia afuera.';
  }
}

/// Banner shown while the drawn polygon cannot be sent.
class GeofenceProblemBanner extends StatelessWidget {
  const GeofenceProblemBanner({super.key});

  @override
  Widget build(BuildContext context) {
    final geofence = context.watch<GeofenceProvider>();
    final problem = geofence.problem;
    if (problem == null || geofence.points.isEmpty) {
      return const SizedBox.shrink();
    }

    final isHint = problem == GeofenceProblem.tooFewPoints;
    return _NoticeBox(
      icon: isHint ? Icons.info_outline : Icons.warning_amber_rounded,
      text: geofenceProblemMessage(problem),
      background: isHint ? RoboColors.warningChip : RoboColors.errorChip,
      foreground: isHint ? RoboColors.textSecondary : RoboColors.error,
    );
  }
}

/// Status of the fence as reported by the robot (robot/status "geofence").
/// Hidden while the robot does not report it.
class GeofenceRobotStatus extends StatelessWidget {
  const GeofenceRobotStatus({super.key});

  @override
  Widget build(BuildContext context) {
    final status = context.watch<RobotStateProvider>().currentStatus.geofenceStatus;
    if (status == null) return const SizedBox.shrink();

    final String text;
    final bool ok = status == 'OK';
    final bool rejected = status.startsWith('INVALID');
    if (ok) {
      text = 'Cerco activo en el robot';
    } else if (rejected) {
      final reason = status.contains(':') ? status.substring(status.indexOf(':') + 1) : '';
      text = reason.isEmpty
          ? 'El robot rechazó el cerco'
          : 'El robot rechazó el cerco: $reason';
    } else if (status == 'WAITING_CONVERSION') {
      text = 'Robot: esperando GPS para ubicar el cerco';
    } else if (status == 'NO_POSE') {
      text = 'Robot: sin posición todavía';
    } else if (status == 'NO_POLYGON') {
      text = 'El robot no tiene un cerco cargado';
    } else {
      text = 'Estado del cerco en el robot: $status';
    }

    return _NoticeBox(
      icon: ok
          ? Icons.check_circle_outline
          : (rejected ? Icons.error_outline : Icons.hourglass_empty),
      text: text,
      background: ok
          ? RoboColors.successChip
          : (rejected ? RoboColors.errorChip : RoboColors.warningChip),
      foreground: ok
          ? RoboColors.successDark
          : (rejected ? RoboColors.error : RoboColors.textSecondary),
    );
  }
}

class _NoticeBox extends StatelessWidget {
  final IconData icon;
  final String text;
  final Color background;
  final Color foreground;

  const _NoticeBox({
    required this.icon,
    required this.text,
    required this.background,
    required this.foreground,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: background,
        borderRadius: BorderRadius.circular(14),
      ),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Icon(icon, size: 16, color: foreground),
          const SizedBox(width: 6),
          Expanded(
            child: Text(
              text,
              style: TextStyle(
                fontSize: 11,
                fontWeight: FontWeight.w700,
                color: foreground,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
