import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/geofence_provider.dart';
import '../theme/robo_theme.dart';

class GeofenceToolbar extends StatelessWidget {
  const GeofenceToolbar({super.key});

  @override
  Widget build(BuildContext context) {
    final geofence = context.watch<GeofenceProvider>();

    return Container(
      padding: const EdgeInsets.fromLTRB(16, 8, 8, 8),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.95),
        borderRadius: BorderRadius.circular(22),
        boxShadow: const [
          BoxShadow(
            color: Color(0x26000000),
            blurRadius: 14,
            offset: Offset(0, 4),
          ),
        ],
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text(
            '${geofence.points.length} pts',
            style: const TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w800,
              color: RoboColors.darkGreen,
            ),
          ),
          const SizedBox(width: 6),
          _ToolbarButton(
            icon: Icons.undo,
            tooltip: 'Deshacer',
            background: RoboColors.border,
            iconColor: RoboColors.textTertiary,
            onTap: geofence.points.isEmpty ? null : geofence.undo,
          ),
          _ToolbarButton(
            icon: Icons.clear,
            tooltip: 'Limpiar',
            background: RoboColors.errorChip,
            iconColor: RoboColors.error,
            onTap: geofence.points.isEmpty ? null : geofence.clear,
          ),
          _ToolbarButton(
            icon: Icons.send,
            tooltip: 'Enviar geofence',
            background:
                geofence.canSend ? RoboColors.darkGreen : RoboColors.disabled,
            iconColor: Colors.white,
            onTap: geofence.canSend
                ? () {
                    final ok = geofence.sendGeofence();
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: Text(ok
                            ? 'Geofence enviado al robot'
                            : 'Mínimo 3 puntos requeridos'),
                        duration: const Duration(seconds: 2),
                      ),
                    );
                  }
                : null,
          ),
        ],
      ),
    );
  }
}

class _ToolbarButton extends StatelessWidget {
  final IconData icon;
  final String tooltip;
  final Color background;
  final Color iconColor;
  final VoidCallback? onTap;

  const _ToolbarButton({
    required this.icon,
    required this.tooltip,
    required this.background,
    required this.iconColor,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 2),
      child: Tooltip(
        message: tooltip,
        child: InkWell(
          onTap: onTap,
          customBorder: const CircleBorder(),
          child: Opacity(
            opacity: onTap == null ? 0.4 : 1.0,
            child: Container(
              width: 32,
              height: 32,
              decoration: BoxDecoration(color: background, shape: BoxShape.circle),
              child: Icon(icon, size: 15, color: iconColor),
            ),
          ),
        ),
      ),
    );
  }
}
