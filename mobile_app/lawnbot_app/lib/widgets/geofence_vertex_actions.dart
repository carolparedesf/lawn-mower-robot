import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/geofence_provider.dart';
import '../theme/robo_theme.dart';
import '../utils/geo.dart';

/// Polygon size in meters, shown while drawing.
class GeofenceSizeLabel extends StatelessWidget {
  const GeofenceSizeLabel({super.key});

  @override
  Widget build(BuildContext context) {
    final size = polygonSize(context.watch<GeofenceProvider>().points);
    if (size == null) return const SizedBox.shrink();
    final text = 'Tamaño: ${size.eastWest.toStringAsFixed(1)} m (E-O) × '
        '${size.northSouth.toStringAsFixed(1)} m (N-S)'
        '${size.area > 0 ? ' · ${size.area.toStringAsFixed(0)} m²' : ''}';
    return Container(
      key: const Key('geofence_size'),
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.92),
        borderRadius: BorderRadius.circular(14),
      ),
      child: Text(
        text,
        style: const TextStyle(
          fontSize: 11,
          fontWeight: FontWeight.w700,
          color: RoboColors.darkGreen,
        ),
      ),
    );
  }
}

/// Extra ways to create vertices besides tapping the map.
class GeofenceVertexActions extends StatelessWidget {
  /// Null when the robot position is missing or stale: the button is disabled.
  final VoidCallback? onAddAtRobot;
  final VoidCallback onRectangle;
  final bool pickingCorner;

  const GeofenceVertexActions({
    super.key,
    required this.onAddAtRobot,
    required this.onRectangle,
    required this.pickingCorner,
  });

  @override
  Widget build(BuildContext context) {
    return Wrap(
      spacing: 8,
      runSpacing: 6,
      alignment: WrapAlignment.center,
      children: [
        ActionChip(
          key: const Key('add_vertex_at_robot'),
          avatar: const Icon(Icons.my_location, size: 16),
          label: const Text('Vértice en el robot'),
          tooltip: onAddAtRobot == null
              ? 'Requiere una posición del robot de menos de 3 s'
              : 'Agregar un vértice en la posición actual del robot',
          backgroundColor: Colors.white,
          onPressed: onAddAtRobot,
        ),
        ActionChip(
          key: const Key('rectangle_by_measures'),
          avatar: Icon(pickingCorner ? Icons.close : Icons.crop_square, size: 16),
          label: Text(pickingCorner ? 'Cancelar rectángulo' : 'Rectángulo por medidas'),
          backgroundColor: pickingCorner ? RoboColors.amberChip : Colors.white,
          onPressed: onRectangle,
        ),
      ],
    );
  }
}
